#include "tron2_controllers/ControllerBase.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace tron2_controller
{

bool ControllerBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
    if (!loadRLCfg(nh))
    {
        ROS_ERROR("Failed to load RL config");
        return false;
    }
    if (!loadModel(nh))
    {
        ROS_ERROR("Failed to load ONNX model");
        return false;
    }

    auto *hybridJointInterface = robot_hw->get<robot_common::HybridJointInterface>();
    for (const auto &name : cfg_.jointNames)
    {
        hybridJointHandles_.push_back(hybridJointInterface->getHandle(name));
    }

    auto *imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
    nh.getParam("imu_name", tron2ImuHandleName_);
    imuSensorHandle_ = imuInterface->getHandle(tron2ImuHandleName_);

    cmdVelSub_ = nh.subscribe("/cmd_vel", 1, &ControllerBase::cmdVelCallback, this);
    modeSub_ = nh.subscribe("/tron2_controller/set_mode", 1, &ControllerBase::modeCallback, this);
    tron2Command_.setZero();

    observations_.setZero(cfg_.policyObsSize);
    actions_.setZero(cfg_.actionsSize);
    lastActions_.setZero(cfg_.actionsSize);
    encoderOut_.setZero(cfg_.encoderOutputSize);
    proprioHistoryBuffer_.setZero(cfg_.obsHistoryLength * cfg_.policyObsSize);

    nh.getParam("policy_dt", tron2PolicyDt_);
    nh.getParam("hw_control_dt", tron2HwControlDt_);

    nh.getParam("/Tron2Cfg/stand_mode/stand_duration", standDuration_);
    tron2InitJointAngles_.setZero(cfg_.jointNames.size());

    ROS_INFO("Controller initialized: variant=%s imu=%s, policyDt=%.4f, decimation=%d, policyObsSize=%d, actionsSize=%d",
             robotVariant_.c_str(), tron2ImuHandleName_.c_str(), tron2PolicyDt_, cfg_.decimation, cfg_.policyObsSize, cfg_.actionsSize);

    return true;
}

void ControllerBase::starting(const ros::Time &)
{
    loopCount_ = 0;
    isFirstObs_ = true;
    footGaitPhaseIndex_ = 0.0;
    mode_ = STAND;
    standPercent_ = 0.0;
    for (size_t i = 0; i < hybridJointHandles_.size(); i++)
    {
        tron2InitJointAngles_(i) = hybridJointHandles_[i].getPosition();
    }
    lastActions_.setZero();
    actions_.setZero();
    proprioHistoryBuffer_.setZero();
    tron2Command_.setZero();
}

void ControllerBase::update(const ros::Time &, const ros::Duration &)
{
    if (mode_ == IDLE)
    {
        for (int i = 0; i < cfg_.actionsSize; i++)
        {
            double current_pos = hybridJointHandles_[i].getPosition();
            hybridJointHandles_[i].setCommand(current_pos, 0.0, 0.0, cfg_.kd[i], 0.0, 0);
        }
        return;
    }

    if (mode_ == STAND)
    {
        if (standPercent_ < 1.0)
        {
            for (int i = 0; i < cfg_.actionsSize; i++)
            {
                double pos_des = tron2InitJointAngles_(i) * (1.0 - standPercent_) + cfg_.defaultDofPos[i] * standPercent_;
                hybridJointHandles_[i].setCommand(pos_des, 0.0, cfg_.kp[i], cfg_.kd[i], 0.0, 0);
            }
            standPercent_ += tron2HwControlDt_ / standDuration_;
        }
        else
        {
            for (int i = 0; i < cfg_.actionsSize; i++)
            {
                hybridJointHandles_[i].setCommand(cfg_.defaultDofPos[i], 0.0, cfg_.kp[i], cfg_.kd[i], 0.0, 0);
            }
        }
        return;
    }

    loopCount_++;
    if (loopCount_ % cfg_.decimation == 0)
    {
        computeObservation();
        updateObsHistory();
        computeEncoder();
        computeActions();

        // Limit action range
        float actionMin = -cfg_.clipActions;
        float actionMax = cfg_.clipActions;
        std::transform(actions_.data(), actions_.data() + actions_.size(), actions_.data(),
                       [actionMin, actionMax](float x) { return std::max(actionMin, std::min(actionMax, x)); });

        lastActions_ = actions_;
    }

    applyActions();
}

bool ControllerBase::loadRLCfg(ros::NodeHandle &nh)
{
    const std::string ns = "/Tron2Cfg";
    nh.getParam(ns + "/robot_variant", robotVariant_);
    isWheelfootVariant_ = (robotVariant_.find("WF_") != std::string::npos) ||
                          (robotVariant_.find("WHEELFOOT") != std::string::npos);
    isSolefootVariant_ = (robotVariant_.find("SF_") != std::string::npos) ||
                         (robotVariant_.find("SOLEFOOT") != std::string::npos);
    nh.getParam(ns + "/init_state/joint_names", cfg_.jointNames);
    if (cfg_.jointNames.empty())
    {
        ROS_ERROR("No joint_names found in %s", (ns + "/init_state/joint_names").c_str());
        return false;
    }

    cfg_.sensorJointNames = cfg_.jointNames;
    nh.getParam(ns + "/init_state/sensor_joint_names", cfg_.sensorJointNames);

    const int numJoints = static_cast<int>(cfg_.jointNames.size());
    cfg_.defaultDofPos.resize(numJoints, 0.0);
    cfg_.defaultDofPosSensor.resize(numJoints, 0.0);

    for (int i = 0; i < numJoints; i++)
    {
        nh.getParam(ns + "/init_state/default_joint_angle/" + cfg_.jointNames[i], cfg_.defaultDofPos[i]);
        nh.getParam(ns + "/init_state/default_joint_angle/" + cfg_.sensorJointNames[i], cfg_.defaultDofPosSensor[i]);
    }

    nh.getParam(ns + "/control/joint_control_modes", cfg_.jointModes);

    double legStiffness, legDamping, lightStiffness, lightDamping;
    double legTorqueLimit, lightTorqueLimit;
    double wheelDamping, wheelTorqueLimit;
    nh.getParam(ns + "/control/leg_joint_stiffness", legStiffness);
    nh.getParam(ns + "/control/leg_joint_damping", legDamping);
    nh.getParam(ns + "/control/light_joint_stiffness", lightStiffness);
    nh.getParam(ns + "/control/light_joint_damping", lightDamping);
    nh.getParam(ns + "/control/leg_joint_torque_limit", legTorqueLimit);
    nh.getParam(ns + "/control/light_joint_torque_limit", lightTorqueLimit);
    nh.getParam(ns + "/control/wheel_joint_damping", wheelDamping);
    nh.getParam(ns + "/control/wheel_joint_torque_limit", wheelTorqueLimit);

    cfg_.kp.resize(numJoints, 0.0);
    cfg_.kd.resize(numJoints, 0.0);
    cfg_.torqueLimits.resize(numJoints, 0.0);
    for (int i = 0; i < numJoints; i++)
    {
        const std::string &name = cfg_.jointNames[i];
        const bool isWheel = name.find("wheel_") != std::string::npos;
        const bool isYaw = name.find("yaw_") != std::string::npos;
        const bool isAnklePitch = name.find("ankle_pitch_") != std::string::npos;
        if (isWheel)
        {
            cfg_.kp[i] = 0.0;
            cfg_.kd[i] = wheelDamping;
            cfg_.torqueLimits[i] = wheelTorqueLimit;
        }
        else if (isYaw || isAnklePitch)
        {
            cfg_.kp[i] = lightStiffness;
            cfg_.kd[i] = lightDamping;
            cfg_.torqueLimits[i] = lightTorqueLimit;
        }
        else
        {
            cfg_.kp[i] = legStiffness;
            cfg_.kd[i] = legDamping;
            cfg_.torqueLimits[i] = legTorqueLimit;
        }
    }

    nh.getParam(ns + "/control/action_scale_pos", cfg_.actionScalePos);
    nh.getParam(ns + "/control/action_scale_vel", cfg_.actionScaleVel);
    nh.getParam(ns + "/control/decimation", cfg_.decimation);
    nh.getParam(ns + "/normalization/clip_scales/clip_observations", cfg_.clipObs);
    nh.getParam(ns + "/normalization/clip_scales/clip_actions", cfg_.clipActions);

    nh.getParam(ns + "/size/actions_size", cfg_.actionsSize);
    nh.getParam(ns + "/size/policy_obs_size", cfg_.policyObsSize);
    nh.getParam(ns + "/size/commands_obs_size", cfg_.commandsObsSize);
    nh.getParam(ns + "/size/encoder_input_size", cfg_.encoderInputSize);
    nh.getParam(ns + "/size/encoder_output_size", cfg_.encoderOutputSize);
    nh.getParam(ns + "/size/policy_input_size", cfg_.policyInputSize);
    nh.getParam(ns + "/size/obs_history_length", cfg_.obsHistoryLength);

    nh.getParam(ns + "/gait/enabled", cfg_.gaitEnabled);
    nh.getParam(ns + "/gait/frequency", cfg_.gaitFrequency);
    nh.getParam(ns + "/gait/offset", cfg_.gaitOffset);
    nh.getParam(ns + "/gait/duration", cfg_.gaitDuration);
    nh.getParam(ns + "/gait/swing_height", cfg_.gaitSwingHeight);

    nh.getParam(ns + "/commands/max/lin_vel_x", cfg_.maxCmdVx);
    nh.getParam(ns + "/commands/max/lin_vel_y", cfg_.maxCmdVy);
    nh.getParam(ns + "/commands/max/ang_vel_yaw", cfg_.maxCmdYaw);

    std::unordered_map<std::string, int> sensorIdxByName;
    for (int i = 0; i < static_cast<int>(cfg_.sensorJointNames.size()); i++)
    {
        sensorIdxByName[cfg_.sensorJointNames[i]] = i;
    }
    cfg_.actionToSensorIdx.resize(numJoints);
    for (int i = 0; i < numJoints; i++)
    {
        cfg_.actionToSensorIdx[i] = sensorIdxByName[cfg_.jointNames[i]];
    }

    cfg_.wheelfootNonWheelSensorIdx.clear();
    for (int i = 0; i < static_cast<int>(cfg_.sensorJointNames.size()); i++)
    {
        if (cfg_.sensorJointNames[i].find("wheel_") == std::string::npos)
        {
            cfg_.wheelfootNonWheelSensorIdx.push_back(i);
        }
    }

    return true;
}

bool ControllerBase::loadModel(ros::NodeHandle &nh)
{
    std::string policyFile;
    std::string encoderFile;
    if (!nh.getParam("/policyFile", policyFile) || !nh.getParam("/encoderFile", encoderFile))
    {
        ROS_ERROR("policyFile or encoderFile param not found");
        return false;
    }

    onnxEnvPtr_ = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Tron2Controller");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetInterOpNumThreads(1);

    policySessionPtr_ = new Ort::Session(*onnxEnvPtr_, policyFile.c_str(), sessionOptions);
    encoderSessionPtr_ = new Ort::Session(*onnxEnvPtr_, encoderFile.c_str(), sessionOptions);

    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t i = 0; i < policySessionPtr_->GetInputCount(); i++)
    {
        policyInputNames_.push_back(policySessionPtr_->GetInputName(i, allocator));
        policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }
    for (size_t i = 0; i < policySessionPtr_->GetOutputCount(); i++)
    {
        policyOutputNames_.push_back(policySessionPtr_->GetOutputName(i, allocator));
    }
    for (size_t i = 0; i < encoderSessionPtr_->GetInputCount(); i++)
    {
        encoderInputNames_.push_back(encoderSessionPtr_->GetInputName(i, allocator));
        encoderInputShapes_.push_back(encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }
    for (size_t i = 0; i < encoderSessionPtr_->GetOutputCount(); i++)
    {
        encoderOutputNames_.push_back(encoderSessionPtr_->GetOutputName(i, allocator));
    }

    return true;
}

void ControllerBase::updateObsHistory()
{
    const int obsSize = cfg_.policyObsSize;
    const int histLen = cfg_.obsHistoryLength;
    if (isFirstObs_)
    {
        for (int t = 0; t < histLen; t++)
        {
            proprioHistoryBuffer_.segment(t * obsSize, obsSize) = observations_;
        }
        isFirstObs_ = false;
        return;
    }

    for (int t = 0; t < histLen - 1; t++)
    {
        proprioHistoryBuffer_.segment(t * obsSize, obsSize) =
            proprioHistoryBuffer_.segment((t + 1) * obsSize, obsSize);
    }
    proprioHistoryBuffer_.segment((histLen - 1) * obsSize, obsSize) = observations_;
}

void ControllerBase::computeEncoder()
{
    auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                 OrtMemType::OrtMemTypeDefault);
    std::vector<int64_t> shape = encoderInputShapes_[0];
    if (shape.size() == 1)
    {
        shape[0] = cfg_.encoderInputSize;
    }
    else if (shape.size() == 2)
    {
        shape[0] = 1;
        shape[1] = cfg_.encoderInputSize;
    }

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, proprioHistoryBuffer_.data(), cfg_.encoderInputSize, shape.data(), shape.size());
    auto output = encoderSessionPtr_->Run(Ort::RunOptions{nullptr},
                                          encoderInputNames_.data(), &inputTensor, 1,
                                          encoderOutputNames_.data(), 1);
    const float *outputData = output[0].GetTensorData<float>();
    for (int i = 0; i < cfg_.encoderOutputSize; i++)
    {
        encoderOut_(i) = outputData[i];
    }
}

void ControllerBase::computeActions()
{
    Eigen::VectorXf policyInput(cfg_.policyInputSize);
    int offset = 0;
    policyInput.segment(offset, cfg_.encoderOutputSize) = encoderOut_;
    offset += cfg_.encoderOutputSize;
    policyInput.segment(offset, cfg_.policyObsSize) = observations_;
    offset += cfg_.policyObsSize;
    policyInput(offset) = static_cast<float>(tron2Command_(0));
    policyInput(offset + 1) = static_cast<float>(tron2Command_(1));
    policyInput(offset + 2) = static_cast<float>(tron2Command_(2));

    auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                                 OrtMemType::OrtMemTypeDefault);
    std::vector<int64_t> shape = policyInputShapes_[0];
    if (shape.size() == 1)
    {
        shape[0] = cfg_.policyInputSize;
    }
    else if (shape.size() == 2)
    {
        shape[0] = 1;
        shape[1] = cfg_.policyInputSize;
    }

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, policyInput.data(), cfg_.policyInputSize, shape.data(), shape.size());
    auto output = policySessionPtr_->Run(Ort::RunOptions{nullptr},
                                         policyInputNames_.data(), &inputTensor, 1,
                                         policyOutputNames_.data(), 1);
    const float *outputData = output[0].GetTensorData<float>();
    for (int i = 0; i < cfg_.actionsSize; i++)
    {
        actions_(i) = std::clamp(outputData[i],
                                 static_cast<float>(-cfg_.clipActions),
                                 static_cast<float>(cfg_.clipActions));
    }
}

void ControllerBase::clipActionsByTorque()
{
    for (int i = 0; i < cfg_.actionsSize; i++)
    {
        if (cfg_.jointModes[i] != "position" || cfg_.kp[i] <= 1e-6)
        {
            continue;
        }
        const double q = hybridJointHandles_[i].getPosition();
        const double dq = hybridJointHandles_[i].getVelocity();
        const double deltaPos = q - cfg_.defaultDofPos[i];
        const double kp = cfg_.kp[i];
        const double kd = cfg_.kd[i];
        const double tl = cfg_.torqueLimits[i];
        const double actMin = (deltaPos + (kd * dq - tl) / kp) / cfg_.actionScalePos;
        const double actMax = (deltaPos + (kd * dq + tl) / kp) / cfg_.actionScalePos;
        actions_(i) = std::clamp(static_cast<double>(actions_(i)), actMin, actMax);
    }
}

void ControllerBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    tron2Command_(0) = std::clamp(msg->linear.x, -cfg_.maxCmdVx, cfg_.maxCmdVx);
    tron2Command_(1) = std::clamp(msg->linear.y, -cfg_.maxCmdVy, cfg_.maxCmdVy);
    tron2Command_(2) = std::clamp(msg->angular.z, -cfg_.maxCmdYaw, cfg_.maxCmdYaw);
}

void ControllerBase::modeCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "WALK" && mode_ != WALK)
    {
        mode_ = WALK;
        loopCount_ = 0;
        isFirstObs_ = true;
        lastActions_.setZero();
        proprioHistoryBuffer_.setZero();
        ROS_INFO("Switching to WALK mode via topic");
    }
    else if (msg->data == "IDLE" && mode_ != IDLE)
    {
        mode_ = IDLE;
        ROS_INFO("Switching to IDLE mode via topic");
    }
}

} // namespace tron2_controller

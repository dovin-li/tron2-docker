#include "tron2_hw/Tron2HW.h"

#include <algorithm>
#include <cmath>

namespace tron2_hw
{

bool Tron2HW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    robot_ = limxsdk::Tron2::getInstance();
    if (robot_ == nullptr)
    {
        ROS_ERROR("Failed to get limxsdk Tron2 instance.");
        return false;
    }

    std::string robotIp;
    robot_hw_nh.param<std::string>("robot_ip", robotIp, "10.192.1.2");
    if (!robot_->init(robotIp))
    {
        ROS_ERROR("limxsdk Tron2::init(\"%s\") failed.", robotIp.c_str());
        return false;
    }
    simMode_ = (robotIp == "127.0.0.1");
    ROS_INFO("limxsdk Tron2 connected to %s (simMode=%d)", robotIp.c_str(), simMode_);

    if (!loadUrdf(root_nh))
    {
        ROS_WARN("Failed to load robot_description, continue with param-based joint setup.");
    }

    if (!loadJointNames(root_nh))
    {
        return false;
    }

    root_nh.param<std::string>("/robot_hw/imu/name", imuName_, "base_imu");
    root_nh.param<std::string>("/robot_hw/imu/frame_id", imuFrameId_, imuName_);
    root_nh.param("/robot_hw/require_calibration_state", requireCalibration_, true);
    if (simMode_)
    {
        requireCalibration_ = false;
    }
    root_nh.param<std::string>("/robot_hw/controller_name", controllerName_, "tron2_controller");
    root_nh.param<std::string>("/robot_hw/controller_manager_ns", controllerManagerNs_, "/tron2_hw/controller_manager");
    root_nh.param("/robot_hw/command_limits/default_kp", defaultKp_, 0.0);
    root_nh.param("/robot_hw/command_limits/default_kd", defaultKd_, 1.0);
    root_nh.param("/Tron2Cfg/commands/max/lin_vel_x", joystickScaleVx_, 0.5);
    root_nh.param("/Tron2Cfg/commands/max/lin_vel_y", joystickScaleVy_, 0.5);
    root_nh.param("/Tron2Cfg/commands/max/ang_vel_yaw", joystickScaleYaw_, 0.5);
    root_nh.param("/robot_hw/use_sdk_joystick_input", useSdkJoystickInput_, true);
    root_nh.param("/robot_hw/debug_joystick_input", debugJoystickInput_, true);

    const int motorNum = static_cast<int>(jointNames_.size());
    jointData_.resize(motorNum);
    robotCmd_ = limxsdk::RobotCmd(motorNum);
    robotStateBuffer_.writeFromNonRT(limxsdk::RobotState(motorNum));
    imuDataBuffer_.writeFromNonRT(limxsdk::ImuData());

    setupInterfaces();
    setupSubscriptions(root_nh, robot_hw_nh);
    setupServices(robot_hw_nh);

    ROS_INFO("Tron2HW initialized: joints=%zu, imu=%s, controller=%s",
             jointNames_.size(), imuName_.c_str(), controllerName_.c_str());
    return true;
}

void Tron2HW::read(const ros::Time &, const ros::Duration &)
{
    const limxsdk::RobotState robotState = *robotStateBuffer_.readFromRT();
    for (size_t i = 0; i < jointData_.size(); ++i)
    {
        jointData_[i].pos_ = robotState.q[i];
        jointData_[i].vel_ = robotState.dq[i];
        jointData_[i].tau_ = robotState.tau[i];
    }

    const limxsdk::ImuData imuMsg = *imuDataBuffer_.readFromRT();
    // limxsdk quaternion is [w, x, y, z], ros imu handle expects [x, y, z, w].
    imuData_.ori_[0] = imuMsg.quat[1];
    imuData_.ori_[1] = imuMsg.quat[2];
    imuData_.ori_[2] = imuMsg.quat[3];
    imuData_.ori_[3] = imuMsg.quat[0];
    imuData_.angularVel_[0] = imuMsg.gyro[0];
    imuData_.angularVel_[1] = imuMsg.gyro[1];
    imuData_.angularVel_[2] = imuMsg.gyro[2];
    imuData_.linearAcc_[0] = imuMsg.acc[0];
    imuData_.linearAcc_[1] = imuMsg.acc[1];
    imuData_.linearAcc_[2] = imuMsg.acc[2];
}

void Tron2HW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    // 1. 填充指令
    for (size_t i = 0; i < jointData_.size(); ++i) {
      robotCmd_.q[i] = static_cast<float>(jointData_[i].posDes_);
      robotCmd_.dq[i] = static_cast<float>(jointData_[i].velDes_);
      robotCmd_.Kp[i] = static_cast<float>(jointData_[i].kp_);
      robotCmd_.Kd[i] = static_cast<float>(jointData_[i].kd_);
      robotCmd_.tau[i] = static_cast<float>(jointData_[i].tauFf_); // 注意：这里是 tauFf_ 而不是 tau_ff_
      robotCmd_.mode[i] = static_cast<float>(jointData_[i].mode_);
    }
  
    // 2. 发布指令（仅在校准完成后）
    // 注意：变量名是 calibrationState_，且建议保留 simMode_ 判断，否则仿真可能无法运行
    if (calibrationState_ == 0) {
      robot_->publishRobotCmd(robotCmd_);
    }
  }

bool Tron2HW::startController()
{
    controller_manager_msgs::ListControllers listControllers;
    if (!listControllersClient_.call(listControllers))
    {
        ROS_ERROR("Failed to call list_controllers service.");
        return false;
    }

    for (const auto &controller : listControllers.response.controller)
    {
        if (controller.name == controllerName_ && controller.state == "running")
        {
            ROS_WARN("Controller %s is already running.", controllerName_.c_str());
            return true;
        }
    }

    controller_manager_msgs::SwitchController sw;
    sw.request.start_controllers.push_back(controllerName_);
    sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
    sw.request.start_asap = false;
    sw.request.timeout = ros::Duration(3.0).toSec();

    if (!switchControllersClient_.call(sw.request, sw.response) || !sw.response.ok)
    {
        ROS_ERROR("Failed to start controller %s.", controllerName_.c_str());
        return false;
    }

    ROS_INFO("Started controller %s.", controllerName_.c_str());
    return true;
}

bool Tron2HW::stopController()
{
    controller_manager_msgs::SwitchController sw;
    sw.request.stop_controllers.push_back(controllerName_);
    sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
    sw.request.start_asap = false;
    sw.request.timeout = ros::Duration(3.0).toSec();

    if (!switchControllersClient_.call(sw.request, sw.response) || !sw.response.ok)
    {
        ROS_WARN("Failed to stop controller %s.", controllerName_.c_str());
        return false;
    }

    ROS_INFO("Stopped controller %s.", controllerName_.c_str());
    return true;
}

bool Tron2HW::loadUrdf(ros::NodeHandle &nh)
{
    std::string urdfString;
    nh.getParam("robot_description", urdfString);
    if (urdfString.empty())
    {
        return false;
    }
    urdfModel_ = std::make_shared<urdf::Model>();
    return urdfModel_->initString(urdfString);
}

bool Tron2HW::loadJointNames(ros::NodeHandle &nh)
{
    if (!nh.getParam("/Tron2Cfg/init_state/joint_names", jointNames_) || jointNames_.empty())
    {
        ROS_ERROR("Missing /Tron2Cfg/init_state/joint_names, cannot setup Tron2HW.");
        return false;
    }
    return true;
}

void Tron2HW::setupInterfaces()
{
    registerInterface(&jointStateInterface_);
    registerInterface(&hybridJointInterface_);
    registerInterface(&imuSensorInterface_);

    for (size_t i = 0; i < jointNames_.size(); ++i)
    {
        auto stateHandle = hardware_interface::JointStateHandle(
            jointNames_[i], &jointData_[i].pos_, &jointData_[i].vel_, &jointData_[i].tau_);
        jointStateInterface_.registerHandle(stateHandle);
        hybridJointInterface_.registerHandle(robot_common::HybridJointHandle(
            stateHandle,
            &jointData_[i].posDes_,
            &jointData_[i].velDes_,
            &jointData_[i].kp_,
            &jointData_[i].kd_,
            &jointData_[i].tauFf_,
            &jointData_[i].mode_));
    }

    imuData_.ori_[3] = 1.0;
    imuData_.oriCov_[0] = imuData_.oriCov_[4] = imuData_.oriCov_[8] = 0.0012;
    imuData_.angularVelCov_[0] = imuData_.angularVelCov_[4] = imuData_.angularVelCov_[8] = 0.0004;
    imuData_.linearAccCov_[0] = imuData_.linearAccCov_[4] = imuData_.linearAccCov_[8] = 0.01;

    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
        imuName_, imuFrameId_, imuData_.ori_, imuData_.oriCov_,
        imuData_.angularVel_, imuData_.angularVelCov_,
        imuData_.linearAcc_, imuData_.linearAccCov_));
}

void Tron2HW::setupSubscriptions(ros::NodeHandle &root_nh, ros::NodeHandle &)
{
    robot_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr &msg)
                                {
                                    robotStateBuffer_.writeFromNonRT(*msg);
                                    firstStateReceived_.store(true);
                                });
    robot_->subscribeImuData([this](const limxsdk::ImuDataConstPtr &msg)
                             { imuDataBuffer_.writeFromNonRT(*msg); });

    root_nh.getParam("/joystick_buttons", tron2JoystickButtonMap_);
    root_nh.getParam("/joystick_axes", tron2JoystickAxisMap_);

    tron2CmdVelPub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    tron2ModePub_ = root_nh.advertise<std_msgs::String>("/tron2_controller/set_mode", 1);
    robot_->subscribeSensorJoy([this](const limxsdk::SensorJoyConstPtr &msg)
                               {
        sdkJoystickMsgReceived_.store(true);
        if (debugJoystickInput_)
        {
            const int l1Idx = tron2JoystickButtonMap_.count("L1") > 0 ? tron2JoystickButtonMap_["L1"] : -1;
            const int xIdx = tron2JoystickButtonMap_.count("X") > 0 ? tron2JoystickButtonMap_["X"] : -1;
            const int yIdx = tron2JoystickButtonMap_.count("Y") > 0 ? tron2JoystickButtonMap_["Y"] : -1;
            const int bIdx = tron2JoystickButtonMap_.count("B") > 0 ? tron2JoystickButtonMap_["B"] : -1;
            const int l1State = (l1Idx >= 0 && l1Idx < static_cast<int>(msg->buttons.size())) ? msg->buttons[l1Idx] : -1;
            const int xState = (xIdx >= 0 && xIdx < static_cast<int>(msg->buttons.size())) ? msg->buttons[xIdx] : -1;
            const int yState = (yIdx >= 0 && yIdx < static_cast<int>(msg->buttons.size())) ? msg->buttons[yIdx] : -1;
            const int bState = (bIdx >= 0 && bIdx < static_cast<int>(msg->buttons.size())) ? msg->buttons[bIdx] : -1;
            ROS_INFO_THROTTLE(0.2,
                              "SDK SensorJoy: buttons=%zu axes=%zu | L1(idx=%d,state=%d) X(idx=%d,state=%d) Y(idx=%d,state=%d) B(idx=%d,state=%d)",
                              msg->buttons.size(), msg->axes.size(),
                              l1Idx, l1State, xIdx, xState, yIdx, yState, bIdx, bState);
            if (tron2JoystickAxisMap_.count("left_vertical") > 0 &&
                tron2JoystickAxisMap_.count("left_horizon") > 0 &&
                tron2JoystickAxisMap_.count("right_horizon") > 0)
            {
                const int lvIdx = tron2JoystickAxisMap_["left_vertical"];
                const int lhIdx = tron2JoystickAxisMap_["left_horizon"];
                const int rhIdx = tron2JoystickAxisMap_["right_horizon"];
                const float lv = (lvIdx >= 0 && lvIdx < static_cast<int>(msg->axes.size())) ? msg->axes[lvIdx] : 0.0f;
                const float lh = (lhIdx >= 0 && lhIdx < static_cast<int>(msg->axes.size())) ? msg->axes[lhIdx] : 0.0f;
                const float rh = (rhIdx >= 0 && rhIdx < static_cast<int>(msg->axes.size())) ? msg->axes[rhIdx] : 0.0f;
                ROS_INFO_THROTTLE(0.2,
                                  "SDK SensorJoy axes: left_vertical=%.3f left_horizon=%.3f right_horizon=%.3f",
                                  lv, lh, rh);
            }
        }
        else
        {
            ROS_INFO_THROTTLE(3.0, "SDK SensorJoy is receiving data (debug log disabled).");
        }

        if (!useSdkJoystickInput_)
        {
            ROS_INFO_THROTTLE(3.0, "SDK SensorJoy control path disabled by /robot_hw/use_sdk_joystick_input=false.");
            return;
        }

        if (tron2JoystickButtonMap_.count("L1") > 0 && tron2JoystickButtonMap_.count("Y") > 0 &&
            msg->buttons[tron2JoystickButtonMap_["L1"]] == 1 && msg->buttons[tron2JoystickButtonMap_["Y"]] == 1)
        {
            if (isControllerRunning())
            {
                std_msgs::String modeMsg;
                modeMsg.data = "WALK";
                tron2ModePub_.publish(modeMsg);
            }
            else
            {
                startController();
            }
        }

        if (tron2JoystickButtonMap_.count("L1") > 0 && tron2JoystickButtonMap_.count("X") > 0 &&
            msg->buttons[tron2JoystickButtonMap_["L1"]] == 1 && msg->buttons[tron2JoystickButtonMap_["X"]] == 1)
        {
            if (isControllerRunning())
            {
                std_msgs::String modeMsg;
                modeMsg.data = "IDLE";
                tron2ModePub_.publish(modeMsg);
                ROS_INFO_THROTTLE(1.0, "Joystick shortcut detected: L1+X -> IDLE");
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "Ignore L1+X because controller %s is not running.", controllerName_.c_str());
            }
        }

        if (tron2JoystickAxisMap_.count("left_vertical") > 0 &&
            tron2JoystickAxisMap_.count("left_horizon") > 0 &&
            tron2JoystickAxisMap_.count("right_horizon") > 0)
        {
            static ros::Time lastPub;
            const ros::Time now = ros::Time::now();
            if ((now - lastPub).toSec() >= (1.0 / 30.0))
            {
                geometry_msgs::Twist twist;
                twist.linear.x = msg->axes[tron2JoystickAxisMap_["left_vertical"]] * joystickScaleVx_;
                twist.linear.y = msg->axes[tron2JoystickAxisMap_["left_horizon"]] * joystickScaleVy_;
                twist.angular.z = msg->axes[tron2JoystickAxisMap_["right_horizon"]] * joystickScaleYaw_;
                tron2CmdVelPub_.publish(twist);
                lastPub = now;
            }
        } });

    if (debugJoystickInput_)
    {
        ros::WallTimerOptions timerOptions(
            ros::WallDuration(2.0),
            [this](const ros::WallTimerEvent &) {
                if (!sdkJoystickMsgReceived_.load())
                {
                    ROS_WARN_THROTTLE(5.0, "No SDK SensorJoy received yet. If you are using pygame only, this is expected.");
                }
            },
            nullptr,
            false,
            true);
        sdkJoystickWatchdogTimer_ = root_nh.createWallTimer(timerOptions);
    }

    robot_->subscribeDiagnosticValue([this](const limxsdk::DiagnosticValueConstPtr &msg)
                                     {
        if (msg->name == "calibration")
        {
            calibrationState_ = msg->code;
            hasCalibrationState_ = true;
            ROS_WARN_THROTTLE(1.0, "Calibration state: %d, message: %s", msg->code, msg->message.c_str());
        }
    });
}

void Tron2HW::setupServices(ros::NodeHandle &robot_hw_nh)
{
    switchControllersClient_ = robot_hw_nh.serviceClient<controller_manager_msgs::SwitchController>(
        controllerManagerNs_ + "/switch_controller");
    listControllersClient_ = robot_hw_nh.serviceClient<controller_manager_msgs::ListControllers>(
        controllerManagerNs_ + "/list_controllers");

    startControllerSrv_ = robot_hw_nh.advertiseService("start_controller", &Tron2HW::onStartController, this);
}

bool Tron2HW::onStartController(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
    const bool ok = startController();
    res.success = ok;
    res.message = ok ? "controller started" : "failed to start controller";
    return true;
}

bool Tron2HW::isControllerRunning()
{
    controller_manager_msgs::ListControllers listControllers;
    if (!listControllersClient_.call(listControllers))
    {
        ROS_WARN_THROTTLE(1.0, "Failed to call list_controllers service.");
        return false;
    }

    for (const auto &controller : listControllers.response.controller)
    {
        if (controller.name == controllerName_ && controller.state == "running")
        {
            return true;
        }
    }
    return false;
}

} // namespace tron2_hw

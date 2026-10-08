#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

    //从ros2_control描述的<hardware><param name="robot_ip">中读取控制器ip,未设置时保留默认值
    auto ip_param = info_.hardware_parameters.find("robot_ip");
    if(ip_param != info_.hardware_parameters.end() && !ip_param->second.empty()){
        _controller_ip = ip_param->second;
    }
    auto comm_param = info_.hardware_parameters.find("servo_comm_type");
    if(comm_param != info_.hardware_parameters.end() && !comm_param->second.empty()){
        try{ _servo_comm_type = std::stoi(comm_param->second); }
        catch(const std::exception&){
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "servo_comm_type '%s' is not an integer, keeping %d",
                comm_param->second.c_str(), _servo_comm_type);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "Controller IP: %s (servo transport: %s)", _controller_ip.c_str(),
                _servo_comm_type == 1 ? "UDP :20007" : "xmlrpc");

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
    }
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);    
        /*
        进入伺服模式。ServoJ 必须由 ServoMoveStart()/ServoMoveEnd() 包裹
        (见厂商示例 fairino-python-sdk linux/example NewTest0609.py TestServoJ)。
        缺少这一步时每个 ServoJ 都会被拒绝并返回错误码 14 —— 这不是故障,
        机器人本身是使能且无报警的。
        Enter servo mode. ServoJ must be bracketed by ServoMoveStart()/ServoMoveEnd();
        without it every ServoJ is rejected with error 14 even though the robot is
        enabled and fault-free.
        */
        _servo_mode_ok = (_ptr_robot->ServoMoveStart(_servo_comm_type) == 0);
        if(_servo_mode_ok){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "伺服模式已开启 (ServoMoveStart) / servo mode entered");
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart 被拒绝 —— 关节状态仍可读取, 但不会下发 ServoJ。"
                "常见原因: 机器人处于手动模式(需自动模式 Mode(0)), 或存在未清除的报警。"
                " ServoMoveStart refused: joint states still readable, but no ServoJ will be"
                " sent. Usual causes: robot in MANUAL mode (needs AUTOMATIC), or an"
                " uncleared fault. Will retry once per second -- flip the pendant to"
                " AUTOMATIC and it recovers without relaunching.");
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    if(_servo_mode_ok){ _ptr_robot->ServoMoveEnd(_servo_comm_type); }//与 on_activate 的 ServoMoveStart 配对
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else{
        hardware_interface::return_type::ERROR;
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        if(!_servo_mode_ok){
            // Retry at 1 Hz so switching the pendant to AUTOMATIC recovers in place.
            if((time - _last_servo_retry).seconds() >= 1.0){
                _last_servo_retry = time;
                _servo_mode_ok = (_ptr_robot->ServoMoveStart(_servo_comm_type) == 0);
                if(_servo_mode_ok){
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "伺服模式已恢复 / servo mode recovered -- ServoJ resuming");
                }
            }
            if(!_servo_mode_ok){
                return hardware_interface::return_type::OK;  // read-only, no command
            }
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }
        //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
        /*
        cmdT 必须是真实的下发周期(见 robot.h): 用 controller_manager 实际的
        period, 而不是写死 0.008 —— 否则控制器按错误的时基插值。
        cmdT must be the real command period (robot.h), so take the controller
        manager's actual period rather than a hardcoded 0.008.
        */
        float cmdT = static_cast<float>(period.seconds());
        if(!(cmdT > 0.0005f && cmdT < 0.05f)){ cmdT = 0.008f; }
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,cmdT,0,0,0,_servo_comm_type);
        if(returncode != 0){
            static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("FairinoHardwareInterface"),
                throttle_clock, 1000, "ServoJ指令下发错误,错误码:%d", returncode);
            _servo_mode_ok = false;  // fall back to the retry path above
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)

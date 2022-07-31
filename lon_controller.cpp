//modules/control/controller/lon_controller.cc
//Line29-36
  //代码定义再apollo::control::命名空间下
  namespace apollo {
  namespace control {
  
  //纵向控制用到了apollo/common的故障码，状态码，轨迹点，车辆状态信息类
  //还用到了apollo/cyber/Time类
  using apollo::common::ErrorCode;
  using apollo::common::Status;
  using apollo::common::TrajectoryPoint;
  using apollo::common::VehicleStateProvider;
  using apollo::cyber::Time;
  
//Line38
  //定义常量重力加速度 9.8
  constexpr double GRA_ACC = 9.8;
  
//Line40-80
  //纵向控制器LonController类的构造函数
  LonController::LonController()
      : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) {
    if (FLAGS_enable_csv_debug) { //是否打开csv debug的功能，默认false，见modules/control/common/control_gflags.cc
             //DEFINE_bool(enable_csv_debug, false
             //如果打开就是创建一个以时间命名的csv文件，写入表头用来存放各种纵向的debug信息
             //默认关闭，不赘述
      time_t rawtime;
      char name_buffer[80];
      std::time(&rawtime);
      std::tm time_tm;
      localtime_r(&rawtime, &time_tm);
      strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
      speed_log_file_ = fopen(name_buffer, "w");
      if (speed_log_file_ == nullptr) {
        AERROR << "Fail to open file:" << name_buffer;
        FLAGS_enable_csv_debug = false;
      }
      if (speed_log_file_ != nullptr) {
        fprintf(speed_log_file_,
          "station_reference,"
          "station_error,"
          "station_error_limited,"
          "preview_station_error,"
          "speed_reference,"
          "speed_error,"
          "speed_error_limited,"
          "preview_speed_reference,"
          "preview_speed_error,"
          "preview_acceleration_reference,"
          "acceleration_cmd_closeloop,"
          "acceleration_cmd,"
          "acceleration_lookup,"
          "speed_lookup,"
          "calibration_value,"
          "throttle_cmd,"
          "brake_cmd,"
          "is_full_stop,"
          "\r\n");
 
        fflush(speed_log_file_);
      }
      AINFO << name_ << " used.";
    }
  }
  
  
//Line82-92
  //关闭log日志文件函数主要fclose实现，本身enable_csv_debug就没打开，略过
  void LonController::CloseLogFile() {
    if (FLAGS_enable_csv_debug) {
      if (speed_log_file_ != nullptr) {
        fclose(speed_log_file_);
        speed_log_file_ = nullptr;
      }
    }
  }
  //Stop()函数调用的就是上面的关闭日志文件函数
  void LonController::Stop() { CloseLogFile(); }
  //LonController类的析构函数调用的就是上面的关闭日志文件函数
  LonController::~LonController() { CloseLogFile(); }
  
//Line94-129
  //纵向控制器的初始化函数
  //输入参数是DependencyInjector类对象指针injector主要是用来获取车辆状态信息的
  //control_conf控制的配置类对象指针
  //ControlConf是由control模块下的proto文件control_conf.proto生成的message类
  Status LonController::Init(std::shared_ptr<DependencyInjector> injector,
                       const ControlConf *control_conf) {
    //将传进来的参数赋给LonController类数据成员control_conf_
    control_conf_ = control_conf;   
    //如果加载的配置为空，则直接返回错误信息
    if (control_conf_ == nullptr) {
      //控制器初始化标志位
      controller_initialized_ = false;
      AERROR << "get_longitudinal_param() nullptr";
      return Status(ErrorCode::CONTROL_INIT_ERROR,
              "Failed to load LonController conf");
    }
    //将传进来的参数车辆状态信息injector赋给LonController类数据成员injector_
    injector_ = injector;
    //纵向控制器的配置加载，实际上读取的是modules/control/conf/control_conf.pb.txt下
    //的lon_controller_conf里的内容
    const LonControllerConf &lon_controller_conf =
        control_conf_->lon_controller_conf();
    //从纵向控制器的配置里读取控制周期ts
    double ts = lon_controller_conf.ts();
    //是否打开超前滞后控制器从纵向控制器配置里读取到
    //modules/control/conf/control_conf.pb.txt下lon_controller_conf里enable_reverse_leadlag_compensation
    //默认false
    bool enable_leadlag =
        lon_controller_conf.enable_reverse_leadlag_compensation();
    
    //位置PID控制器的初始化从modules/control/conf/control_conf.pb.txt->lon_controller_conf->station_pid_conf读取
    station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
    //速度控制器的初始化也是加载control_conf.pb.txt->lon_controller_conf里的相关参数
    speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
 
    //如果打开超前滞后控制器，因为默认关闭，略过
    if (enable_leadlag) {
      station_leadlag_controller_.Init(
    lon_controller_conf.reverse_station_leadlag_conf(), ts);
      speed_leadlag_controller_.Init(
    lon_controller_conf.reverse_speed_leadlag_conf(), ts);
    }
    //车辆的参数从VehicleConfigHelper类的成员函数GetConfig()加载
    //参见VehicleConfigHelper.cc Line33可知 Init() { Init(FLAGS_vehicle_config_path); }
    //vehicle_config_path在modules/common/configs/config_gflags.cc定义gflags理解成一个全局变量前面加Flags_可取出其值
    //DEFINE_string(vehicle_config_path, "/apollo/modules/common/data/vehicle_param.pb.txt"这个就是车辆参数配置文件路径
    vehicle_param_.CopyFrom( //从上述路径拷贝车辆参数配置到vehicle_param_(LonController类的数据成员)
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
 
    //从modules/control/conf/control_conf.pb.txt->lon_controller_conf里加载数字滤波器的参数
    //加载滤波器参数到LonController类的成员变量digital_filter_pitch_angle_
    //这个滤波器是用于对pitch角(车辆的俯仰角)进行滤波，pitch角是用于对车辆控制时的坡道补偿，后面介绍
    SetDigitalFilterPitchAngle(lon_controller_conf);
 
    //从modules/control/conf/control_conf.pb.txt->lon_controller_conf里加载标定表(不同车速下，加速度到油门/制动标定表)
    //加载的标定表放入control_interpolation_中
    LoadControlCalibrationTable(lon_controller_conf);
    //执行到这里，纵向控制器被初始化的标志位置为true，这个标志位也属于LonController类的数据成员
    controller_initialized_ = true;
 
    return Status::OK();
  }
 
//Line131-154
  //SetDigitalFilterPitchAngle()函数就是上面Init函数末尾调用的设置数字滤波器的函数的具体定义
  //输入参数是纵向控制器的配置对象
  void LonController::SetDigitalFilterPitchAngle(
      const LonControllerConf &lon_controller_conf) {   
    //配置中读到的参数滤波器的截止频率存到变量cutoff_freq中
    double cutoff_freq =  
        lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
    //配置中读到的参数滤波器的采样周期存到变量ts中
    double ts = lon_controller_conf.ts();
    //将ts，cutoff_freq作为输入参数调用函数SetDigitalFilter()
    //将参数全部设置到LonController类的数据成员滤波器类对象digital_filter_pitch_angle_中
    SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
  }
  
  //LoadControlCalibrationTable()函数就是上面Init函数末尾调用的加载标定表的函数的具体定义
  //输入参数是纵向控制器的配置对象
  void LonController::LoadControlCalibrationTable(
      const LonControllerConf &lon_controller_conf) {
    //lon_controller_conf是从控制器配置文件中读出来的
    //首先将从lon_controller_conf中读到的标定表存入control_table中
    //引用变量control_table就是lon_controller_conf.calibration_table()，auto根据等式右边自动设定类型
    //加const修饰防止引用变量别名更改原变量
    const auto &control_table = lon_controller_conf.calibration_table();
    //屏幕上打印标定表成功加载信息
    AINFO << "Control calibration table loaded";
    AINFO << "Control calibration table size is "
    << control_table.calibration_size();
    //定义DataType类型变量xyz
    //Apollo类型定义typedef std::vector<std::tuple<double, double, double>> DataType;
    //DataType实际上就是一个三维容器，存放很多组对应的速度，加速度，油门/制动百分数数据
    Interpolation2D::DataType xyz;
    for (const auto &calibration : control_table.calibration()) {
      //依次control_table下的calibration读入速度，加速度，油门/制动百分数数据压入xyz中
      xyz.push_back(std::make_tuple(calibration.speed(),
                              calibration.acceleration(),
                              calibration.command()));
    }
    //std::unique_ptr指针的清空复位，将LonController类数据成员control_interpolation_清空
    control_interpolation_.reset(new Interpolation2D);
    //control_interpolation_调用Init函数,xyz作为参数输入，
    //将标定表读取到LonController类的数据成员xyz_里(特定车速下的加速度对应的控制量百分数)
    ACHECK(control_interpolation_->Init(xyz))
        << "Fail to load control calibration table";
  }
  
//Line156-369
  //计算纵向控制指令
  //输入定位信息指针localization
  //输入底盘信息指针chassis
  //输入规划轨迹信息指针planning_published_trajectory
  //输出全部都填入ControlCommand类对象指针cmd
  //ControlCommand类是由modules/control/proto/control_cmd.proto里定义的message ControlCommand类
  //由proto文件可以生成类定义的.cc和.h文件
  Status LonController::ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis,
      const planning::ADCTrajectory *planning_published_trajectory,
      control::ControlCommand *cmd) {
    //localization/chassis/planning_published_trajectory都是const只允许读取
    //将输入的定位，底盘信息分别读取到LonController类的数据成员localization_,chassis_里
    localization_ = localization;
    chassis_ = chassis;
 
    //将输入的规划轨迹信息分别读取到LonController类的数据成员trajectory_message_
    trajectory_message_ = planning_published_trajectory;
    if (!control_interpolation_) {
      //如果标定表为空返回错误信息
      AERROR << "Fail to initialize calibration table.";
      return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
              "Fail to initialize calibration table.");
    }
 
    //如果规划轨迹信息指针为空或者轨迹分析器里的序号和轨迹message的序号不相等时复位
    //trajectory_analyzer_是LonController类的数据成员，属于类TrajectoryAnalyzer在apollo/modules/control/common/下定义
    //trajectory_message_也是LonController类数据成员，属于类ADCTrajectory由apollo/modules/planning/proto/planning.proto定义的消息类
    if (trajectory_analyzer_ == nullptr ||
        trajectory_analyzer_->seq_num() !=
      trajectory_message_->header().sequence_num()) {
      trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
    }
    //定义常引用变量lon_controller_conf为control_conf.pb.txtx里读取到配置
    const LonControllerConf &lon_controller_conf =
        control_conf_->lon_controller_conf();
 
    //定义临时变量debug是cmd.debug.simple_lon_debug 
    //因为cmd是指针所以这样访问数据成员，cmd所属类型包含一个数据成员debug专门用来存放调试信息
    auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
    //首先将debug清空
    debug->Clear();
 
    //初始化刹车，油门控制命令为0
    double brake_cmd = 0.0;
    double throttle_cmd = 0.0;
    //初始化采样周期ts从lon_controller_conf里读到，lon_controller_conf也是从control_conf.pb.txt中读取到的
    double ts = lon_controller_conf.ts();
    //preview预览时间=lon_controller_conf读到的纵向预览窗口大小*采样时间ts
    double preview_time = lon_controller_conf.preview_window() * ts;
    //定义 enable_leadlag是否打开超前-滞后控制器从lon_controller_conf-enable_reverse_leadlag_compensation读取到的
    //默认也是false不打开
    bool enable_leadlag =
        lon_controller_conf.enable_reverse_leadlag_compensation();
 
    //如果预览窗口读到的是负数则提示错误信息并返回
    if (preview_time < 0.0) {
      const auto error_msg =
    absl::StrCat("Preview time set as: ", preview_time, " less than 0");
      AERROR << error_msg;
      return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
    }
    //调用计算纵向误差函数，输入参数：
    //trajectory_analyzer_.get()获得轨迹信息指针用于提供轨迹点的速度加速度，匹配点参考点等信息
    //preview_time预览时间
    //ts采样周期
    //debug计算得到的误差放入debug中，纵向误差计算的细节在下面函数定义时再详细介绍
    ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts,
                        debug);
    
    //定义临时变量station_error_limit纵向位置误差限制
    //从配置文件control_conf.pb.txt-lon_controller_conf-station_error_limit中读取到
    //默认是2.0m
    double station_error_limit = lon_controller_conf.station_error_limit();
    //定义临时变量station_error_limited为限幅后的纵向位置误差
    double station_error_limited = 0.0;
    //如果打开速度-位置预览，FLAGS_enable_speed_station_preview就是
    //从apollo/modules/control/common/control_gflags.cc中取出全局变量enable_speed_station_preview
    //这种前面加FLAGS_取出全局变量值的用法是由gflags库定义，这里不赘述
    //默认为true打开speed_station_preview
    if (FLAGS_enable_speed_station_preview) {
      //基本概念 预览点：当前时间加上预览时间在轨迹上对应的点，车辆将要到达的纵向位置
      //基本概念 参考点：当前时间在轨迹上对应的点，车辆此刻应该到达的纵向位置
      //基本概念 匹配点：当前时间在轨迹上对应的点，车辆此刻应该到达的纵向位置
      //有两种位置误差
      //第一种, preview_station_error=预览点纵向位置 - 匹配点纵向位置
      //第二种, station_error=参考点纵向位置-匹配点纵向位置
      //如果打开此开关则采用第一种纵向位置误差，纵向误差的计算结果都是存在debug里
      station_error_limited =
    common::math::Clamp(debug->preview_station_error(),
                        -station_error_limit, station_error_limit);
    } else {
      //否则采用第二种纵向位置误差，纵向误差的计算结果都是存在debug里
      station_error_limited = common::math::Clamp(
    debug->station_error(), -station_error_limit, station_error_limit);
    }
 
    //如果轨迹msg信息指针里的档位信息是R档
    if (trajectory_message_->gear() == canbus::Chassis::GEAR_REVERSE) { 
      //从配置文件加载lon_controller_conf里的倒车PID参数配置加载到位置PID控制器对象station_pid_controller_
      //station_pid_controller_是LonController类里的数据成员，是PIDController类对象
      station_pid_controller_.SetPID(//将配置参数加载到对象里
    lon_controller_conf.reverse_station_pid_conf());
      //speed_pid_controller_是LonController类里的数据成员，是PIDController类对象
      speed_pid_controller_.SetPID(lon_controller_conf.reverse_speed_pid_conf());//将配置参数加载到对象里
      //如果打开enable_leadlag超前滞后控制器，默认是关闭直接略过
      //lon_controller_conf.enable_reverse_leadlag_compensation去control_conf.pb.txt里查看
      if (enable_leadlag) {
        station_leadlag_controller_.SetLeadlag(
      lon_controller_conf.reverse_station_leadlag_conf());
        speed_leadlag_controller_.SetLeadlag(
      lon_controller_conf.reverse_speed_leadlag_conf());
      }
      //若非R档 且 当前车速<=高低速切换的转换速度switch_speed
      //switch_speed也是从控制配置文件lon_controller_conf里加载的
    } else if (injector_->vehicle_state()->linear_velocity() <=
         lon_controller_conf.switch_speed()) {
      //速度PID控制器对象speed_pid_controller_加载控制配置文件中低速PID参数
      speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
    } else {
      //否则速度PID控制器对象加载控制配置文件中高速PID参数，通常低速PID参数要更大些
      speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
    }
    
    //非R档情况下，定义临时变量speed_offset速度偏差
    //速度偏差=位置控制器根据(限幅后位置误差，采样周期)计算出控制量即速度
    double speed_offset =
        //Control()是PID控制器的成员函数，根据PID控制器原理计算控制量速度作为speed_offset
        station_pid_controller_.Control(station_error_limited, ts);
    if (enable_leadlag) { //如果打开超前-滞后控制器，默认不打开，略过
      speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
    }
 
    //定义一个临时变量速度控制器的输入speed_controller_input为0
    double speed_controller_input = 0.0;
    //从配置文件加载速度控制器输入限幅control_conf.pb.txt--lon_controller_conf--speed_controller_input_limit
    //速度控制器输入就是PID的输入
    double speed_controller_input_limit =
        lon_controller_conf.speed_controller_input_limit();
    
    //注意这个是limited而不是limit指的是经过限幅之后的速度控制器的输入
    //定义临时变量限幅后的速度控制器输入speed_controller_input_limited为0 
    double speed_controller_input_limited = 0.0;
    //打开speed_station_preview?从control_gflags.cc看是默认打开的
    if (FLAGS_enable_speed_station_preview) {
      //打开的话速度控制器的输入 = 位置控制器计算出的speed_offset + 当前时间向前加上预览时间在轨迹上的对应点的速度和当前车速的偏差
      speed_controller_input = speed_offset + debug->preview_speed_error();
    } else {
      //不打开的话速度控制器的输入 = 位置控制器计算出的speed_offset + 参考点车速和当前车速的偏差
      speed_controller_input = speed_offset + debug->speed_error();
    }
    //计算得到的速度控制器的输入再进行限幅
    speed_controller_input_limited =
        common::math::Clamp(speed_controller_input, -speed_controller_input_limit,
                      speed_controller_input_limit);
 
    //定义临时变量acceleration_cmd_closeloop闭环加速度指令初始化为0
    double acceleration_cmd_closeloop = 0.0;
 
    //闭环的加速度指令就等于速度PID控制器根据速度控制器的输入，以及采样周期去计算，PID控制器原理不再赘述
    acceleration_cmd_closeloop =
        speed_pid_controller_.Control(speed_controller_input_limited, ts);
    //将速度PID控制器中积分器的饱和状态设置到debug.pid_saturation_status里
    debug->set_pid_saturation_status(
        speed_pid_controller_.IntegratorSaturationStatus());
    //如果打开超前滞后控制器，默认关闭，略过
    if (enable_leadlag) {
      acceleration_cmd_closeloop =
    speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
      debug->set_leadlag_saturation_status(
    speed_leadlag_controller_.InnerstateSaturationStatus());
    }
 
    //定义斜坡补偿加速度 = (重力加速度 * 车辆俯仰角的正弦值)再经过数字滤波器滤波得到斜坡加速度补偿
    //slope_offset_compensation是临时变量
    double slope_offset_compensation = digital_filter_pitch_angle_.Filter(
        GRA_ACC * std::sin(injector_->vehicle_state()->pitch()));
 
    //判断坡道补偿加速度是否为非数NaN，当浮点数过小下溢就可能出现NaN非数
    if (std::isnan(slope_offset_compensation)) {
      slope_offset_compensation = 0;
    }
    
    //将斜坡补偿加速度设置到debug里，debug.slope_offset_compensation=slope_offset_compensation protobuf
    debug->set_slope_offset_compensation(slope_offset_compensation);
 
    //定义1个临时变量acceleration_cmd
    //总的加速度指令 = 闭环加速度指令 + 预览参考加速度 + 坡道补偿加速度(如果打开坡道补偿的话)
    double acceleration_cmd =
        acceleration_cmd_closeloop + debug->preview_acceleration_reference() +
        FLAGS_enable_slope_offset * debug->slope_offset_compensation();
    //设置debug的is_full_stop标志位为false,proto类数据成员的赋值方式
    debug->set_is_full_stop(false);
    //获取停车点的一个函数，后面介绍，找到当前规划模块发布的轨迹msg里的第一个v,a都小于一个很小值的点作为停车点
    //找到的这个停车点的纵向位置和当前车辆纵向位置的偏差设置到debug里面去，debug.path_remain()
    GetPathRemain(debug);
 
    // 在快要停车的阶段，用一个固定的standstill的减速度代替刹车控制指令
    //简而言之就是快到停车点时给一个固定-0.3m/s^2的减速度(数值控制配置里设置)
    //trajectory_message_是由apollo/modules/planning/planning.proto定义的message ADCTrajectory类对象
    if ((trajectory_message_->trajectory_type() ==    //轨迹类型还有路径后退，速度后退等参见planning.proto
         apollo::planning::ADCTrajectory::NORMAL) &&  //当轨迹msg中的type是normal(估计应该是向前的正常轨迹) 且
        ((std::fabs(debug->preview_acceleration_reference()) <= //预览点的加速度<=控制配置里的停车时最大允许加速度
        control_conf_->max_acceleration_when_stopped() && //见control_conf.pb.txt默认为0.01    
    std::fabs(debug->preview_speed_reference()) <=        //且 预览点速度小于车辆参数中的最大允许停车速度
        vehicle_param_.max_abs_speed_when_stopped()) ||   //见apollo/modules/common/vehicle_param.pb.txt 默认为0.2
         std::abs(debug->path_remain()) <           //或当前纵向位置到停止点纵向位置偏差小于控制配置
       control_conf_->max_path_remain_when_stopped())) {//里max_path_remain_when_stopped默认0.3m  
          //总结上述条件，轨迹类型normal且(预览点加速度小于阈值且预览点速度小于阈值)或到停车点纵向偏差<阈值    
          //符合条件时，加速度指令按如下方式计算                                
      acceleration_cmd =  //R档略过
    (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
        ? std::max(acceleration_cmd,
                   -lon_controller_conf.standstill_acceleration())
                    //非R档取控制器计算加速度指令，standstill减速度中最小值
                    //standstill减速度见控制配置control_conf.pb.txt--lon_controller_conf--standstill_acceleration
             //standstill_acceleration默认为-0.3 m/s^2
        : std::min(acceleration_cmd,
                   lon_controller_conf.standstill_acceleration());
      ADEBUG << "Stop location reached";
      //若正常执行到这里，车辆就已经完全停住了，设置debug.is_full_stop为true
      debug->set_is_full_stop(true);
    }
 
    //定义油门指令的下边界，为 车辆配置里的throttle_deadzone 和 lon_controller_conf配置里的throttle_minimum_action
    //两者中的较大值
    double throttle_lowerbound =
        std::max(vehicle_param_.throttle_deadzone(),
           lon_controller_conf.throttle_minimum_action());
    //定义刹车指令的下边界
    double brake_lowerbound =
        std::max(vehicle_param_.brake_deadzone(),
           lon_controller_conf.brake_minimum_action());
    //定义临时变量calibration_value标定值初始化为0
    double calibration_value = 0.0;
    //要用来查表加速度，若R档为加速度控制指令取反，非R档保持加速度控制指令
    double acceleration_lookup =
        (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
      ? -acceleration_cmd
      : acceleration_cmd;
 
    //是否用预览点速度来查标定表(车速-加速度-控制指令百分数)
    //FLAGS去control_gflags.cc里取出use_preview_speed_for_table的值，默认是false
    if (FLAGS_use_preview_speed_for_table) {  //默认false略过
      calibration_value = control_interpolation_->Interpolate(
    std::make_pair(debug->preview_speed_reference(), acceleration_lookup));
    } else {
      //use_preview_speed_for_table为false的话，查标定表就用chassis里反馈的实际车速去查
      //用速度加速度根据标定表线性插值得到控制量百分数calibration_value
      calibration_value = control_interpolation_->Interpolate(
    std::make_pair(chassis_->speed_mps(), acceleration_lookup));
    }
 
    //如果请求查表加速度>=0
    if (acceleration_lookup >= 0) {
      //如果计算得到的控制百分数>=0
      if (calibration_value >= 0) {
        //设置油门控制百分数，为油门下边界和查表得到的控制百分数之间的较大值
        throttle_cmd = std::max(calibration_value, throttle_lowerbound);
      } else {
      //如果计算得到的控制百分数<0,但是加速度又大于0
        //设置油门控制百分数为油门下边界
        throttle_cmd = throttle_lowerbound;
      }
      //刹车指令为0，如果要加速
      brake_cmd = 0.0;
    } else {
    //如果请求的查表加速度<0,油门指令置0
      throttle_cmd = 0.0;
      //如果计算得到的控制百分数>=0
      if (calibration_value >= 0) {
        //刹车指令就取刹车下边界
        brake_cmd = brake_lowerbound;
      } else {
      //如果计算得到的控制百分数<0 刹车指令就取标定值相反数和刹车下边界里较大值，其实就是取标定值进行限幅不能太小
        brake_cmd = std::max(-calibration_value, brake_lowerbound);
      }
    }
 
    //将被限制的纵向位置误差设置到debug.station_error_limited,指针对象成员->这样访问
    debug->set_station_error_limited(station_error_limited);
    //位置控制器的输出设置到debug.speedoffset
    debug->set_speed_offset(speed_offset);
    //被限幅的速度控制器的输入设置到debug.speed_controller_input_limited
    debug->set_speed_controller_input_limited(speed_controller_input_limited);
    //计算到的加速度指令设置到debug.acceleration_cmd
    debug->set_acceleration_cmd(acceleration_cmd);
    //计算到的油门指令设置到debug.throttle_cmd
    debug->set_throttle_cmd(throttle_cmd);
    //计算到的刹车指令设置到debug.brake_cmd
    debug->set_brake_cmd(brake_cmd);
    //去查标定表的请求加速度设置到debug.acceleration_lookup
    debug->set_acceleration_lookup(acceleration_lookup);
    //去查标定表的速度将chassis反馈值设置到debug.speed_lookup
    debug->set_speed_lookup(chassis_->speed_mps());
    //将标定表中查到的控制百分数值calibration_value设置到debug.calibration_value
    debug->set_calibration_value(calibration_value);
    //将闭环反馈速度控制器计算得到的控制量加速度设置到debug.acceleration_cmd_closeloop
    debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);
 
    //如果打开csv日志记录默认false
    //总之就往csv里写中间各个变量的值用作调试之用
    if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
      fprintf(speed_log_file_,
        "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
        "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
        debug->station_reference(), debug->station_error(),
        station_error_limited, debug->preview_station_error(),
        debug->speed_reference(), debug->speed_error(),
        speed_controller_input_limited, debug->preview_speed_reference(),
        debug->preview_speed_error(),
        debug->preview_acceleration_reference(), acceleration_cmd_closeloop,
        acceleration_cmd, debug->acceleration_lookup(),
        debug->speed_lookup(), calibration_value, throttle_cmd, brake_cmd,
        debug->is_full_stop());
    }
 
    //如果车辆是以加速度驱动，那么可以忽略下面的油门，制动指令值
    cmd->set_throttle(throttle_cmd);
    cmd->set_brake(brake_cmd);
    cmd->set_acceleration(acceleration_cmd);
 
    //如果车辆的纵向速度绝对u值<=车辆参数配置里设定的停车最大速度绝对值
    //简单理解就是车辆的纵向速度小于某阈值或者chassis反馈的档为信息是N档就认为车已经停住了，下发车辆的换档指令
    if (std::fabs(injector_->vehicle_state()->linear_velocity()) <=
      vehicle_param_.max_abs_speed_when_stopped() ||
        chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
      //若车辆处于停车或N档时下发规划发布的轨迹msg里的档位
      cmd->set_gear_location(trajectory_message_->gear());
    } else {
      //若车辆不处于停车且不在N档时下发chassis反馈的车辆实际档位
      cmd->set_gear_location(chassis->gear_location());
    }
          //返回状态码ok
    return Status::OK();
  }
  
//Line377
  //获取LonController的名字name_,是LonController类的数据成员
  //LonController::LonController()
    //   : name_(ControlConf_ControllerType_Name(ControlConf::LON_CONTROLLER)) 
    //在lon_controller.cc Line41 LonController类的构造函数初始化时就初始化name_为"LON_CONTROLLER"
  std::string LonController::Name() const { return name_; }
  
//Line379-461
  //该函数是计算纵向误差的
  //输入参数规划发布轨迹信息TrajectoryAnalyzer类指针对象trajectory_analyzer
  //输入参数preview_time,预览时间，在控制配置文件里面设置
  //输入参数ts，采样时间，在控制配置文件里设置
  //输入参数debug，SimpleLongitudinalDebug类指针对象debug用来存放计算得到纵向误差信息
  //其他函数通过debug指针进行访问这些纵向误差
  void LonController::ComputeLongitudinalErrors(
      const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,
      const double ts, SimpleLongitudinalDebug *debug) {
    // 分解车辆运动到Frenet坐标，就是分为纵向和横向
    // s: 纵向累积走过的距离沿着参考轨迹
    // s_dot: 纵向沿着参考轨迹的速度
    // d: 相对参考轨迹的横向距离
    // d_dot: 横向距离的变化率
    //matched:匹配点，在参考轨迹上距离当前车辆距离最近的点
    //初始化匹配点处的s,s',d,d'
    double s_matched = 0.0;
    double s_dot_matched = 0.0;
    double d_matched = 0.0;
    double d_dot_matched = 0.0;
 
    //定义车辆状态通过LonController类的数据成员injector_去访问其自身的成员函数从而获得车辆当前状态
    auto vehicle_state = injector_->vehicle_state();
    //匹配点为将车辆当前状态的x,y坐标代入去查找轨迹上的最近点
    auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
        vehicle_state->x(), vehicle_state->y());
 
    //轨迹信息将当前点x,y,theta,v以及参考点信息输入，输出当前点的s,d,s',d'
    //简而言之就是将大地坐标系转化为Frenet坐标
    //d是横向偏差，s是累积的弧长即纵向上走过的距离
    //函数参数最后几个都带&，熟悉的套路，引用变量传值，最后带&的几个变量都是待填充函数结果的变量
    trajectory_analyzer->ToTrajectoryFrame(
        vehicle_state->x(), vehicle_state->y(), vehicle_state->heading(),
        vehicle_state->linear_velocity(), matched_point, &s_matched,
        &s_dot_matched, &d_matched, &d_dot_matched);
 
    //定义临时变量当前控制时间=当前时间
    double current_control_time = Time::Now().ToSecond();
    //定义临时变量预览控制时间=当前时间+预览时间
    double preview_control_time = current_control_time + preview_time;
  
    //参考点就是用当前时间去轨迹上查时间最近点
    TrajectoryPoint reference_point =
        trajectory_analyzer->QueryNearestPointByAbsoluteTime(
      current_control_time);
          //预览点就是去轨迹上查预览时间对应的点，就是当前时间向前看一段时间对应轨迹上的点
    TrajectoryPoint preview_point =
        trajectory_analyzer->QueryNearestPointByAbsoluteTime(
      preview_control_time);
 
    //所有的计算结果都是存到debug这个指针对象里
    //debug.matched_point.pathpoint.x=匹配点x  这样表达只是为了简单说明含义，实际指针对象需要通过->访问成员
    debug->mutable_current_matched_point()->mutable_path_point()->set_x(
        matched_point.x());
    //debug.matched_point.pathpoint.y=匹配点y
    debug->mutable_current_matched_point()->mutable_path_point()->set_y(
        matched_point.y());
          //debug.reference_point.pathpoint.x=参考点x        
    debug->mutable_current_reference_point()->mutable_path_point()->set_x(
        reference_point.path_point().x());
    //debug.reference_point.pathpoint.y=参考点y  
    debug->mutable_current_reference_point()->mutable_path_point()->set_y(
        reference_point.path_point().y());
    //debug.preview_point.pathpoint.x=预览点x
    debug->mutable_preview_reference_point()->mutable_path_point()->set_x(
        preview_point.path_point().x());
    debug.preview_point.pathpoint.y=预览点y
    debug->mutable_preview_reference_point()->mutable_path_point()->set_y(
        preview_point.path_point().y());
 
    //打印匹配点，参考点，预览点信息
    ADEBUG << "matched point:" << matched_point.DebugString();
    ADEBUG << "reference point:" << reference_point.DebugString();
    ADEBUG << "preview point:" << preview_point.DebugString();
 
    //航向角误差 = 车辆当前状态航向角 - 匹配点的航向角
    //NormalizeAngle角度的规范化，就是将所有角度规范到-pi,pi
    double heading_error = common::math::NormalizeAngle(vehicle_state->heading() -
                                                  matched_point.theta());
          //纵向速度 = 车辆速度 * cos(当前航向角 - 轨迹上距离最近点航向角)
    double lon_speed = vehicle_state->linear_velocity() * std::cos(heading_error);
    //纵向加速度 = 车辆加速度 * cos(当前航向角 - 轨迹上距离最近点航向角)
    double lon_acceleration =
        vehicle_state->linear_acceleration() * std::cos(heading_error);
    //1-kd就是将大地坐标系转化到Frenet坐标纵向上引入的
    double one_minus_kappa_lat_error = 1 - reference_point.path_point().kappa() *
                                         vehicle_state->linear_velocity() *
                                         std::sin(heading_error);
 
    //计算的相关误差等结果全部存放到指针debug里供类内其他函数调用
    //参考的纵向位置debug.station_reference=参考点的累积弧长
    debug->set_station_reference(reference_point.path_point().s());
    //当前的纵向位置debug.current_station=当前点的累积弧长
    debug->set_current_station(s_matched);
    //纵向位置误差debug.station_error=参考点路径点的累积弧长-匹配点的累积弧长(匹配点就是路径最近点)
    debug->set_station_error(reference_point.path_point().s() - s_matched);
    //速度参考值就是参考点的速度debug.speed_reference
    debug->set_speed_reference(reference_point.v());
    //当前车速就是纵向速度debug.current_speed=lon_speed
    debug->set_current_speed(lon_speed);
    //速度误差就是参考点速度减匹配点速度 debug.speed_error
    debug->set_speed_error(reference_point.v() - s_dot_matched);
    //加速度参考就是参考点的加速度存到debug里
    debug->set_acceleration_reference(reference_point.a());
    //纵向加速度存到debug里
    debug->set_current_acceleration(lon_acceleration);
    //设定加速度误差=参考点加速度-纵向加速度/(1-kd)  1-kd由全局坐标转换到Frenet坐标引入，kappa就是曲率
    debug->set_acceleration_error(reference_point.a() -
                            lon_acceleration / one_minus_kappa_lat_error);
    //参考的加加速度=(参考点加速度-上一时刻的参考加速度)/采样时间
    double jerk_reference =
        (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
    //纵向加加速度=(当前加速度-上一时刻加速度)/采样时间
    double lon_jerk =
        (debug->current_acceleration() - previous_acceleration_) / ts;
    //参考加加速度存到debug
    debug->set_jerk_reference(jerk_reference);
    //当前的加加速度存到debug
    debug->set_current_jerk(lon_jerk);
    //加加速度误差=加加速度参考-纵向加加速度/(1-kd)存到debug里
    debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
    //将此刻的参考加速度赋给上一时刻参考加速度进行迭代
    previous_acceleration_reference_ = debug->acceleration_reference();
    //将当前的加速度赋给上一时刻的当前加速度，这样迭代一般都是为了计算差分来近似导数
    previous_acceleration_ = debug->current_acceleration();
 
          //预览点位置误差=预览点的纵向位置s-匹配点纵向位置s结果存到debug中去
    debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
    //预览点速度误差=预览点的纵向速度v-匹配点纵向速度 结果存到debug中去
    debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
    //预览点速度设置到debug中
    debug->set_preview_speed_reference(preview_point.v());
    //预览点加速度设置到debug中
    debug->set_preview_acceleration_reference(preview_point.a());
  }
  
//Line463-469
  //这个函数是设置数字滤波器，输入采样时间，截至频率
  //digital_filter指针是数字滤波器对象，用ts,cutoff_freq去设置这个滤波器对象，这个对象指针其实是最终的结果
  void LonController::SetDigitalFilter(double ts, double cutoff_freq,
                                 common::DigitalFilter *digital_filter) {
    std::vector<double> denominators;
    std::vector<double> numerators;
    common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
    digital_filter->set_coefficients(denominators, numerators);
  }
  
//Line472-517 end
  //这个函数输入是debug指针，就是去轨迹点上找第一个符合条件的停止点，并将结果存在debug指针
  //在ComputeControlCommand()函数中调用了该函数，目的就是先找到最新发布的轨迹上的第一个停车点，
  //然后快到停车点时，就给一个固定的standstill减速度，配置里默认设置-0.3
  void LonController:: GetPathRemain(SimpleLongitudinalDebug *debug) {
    //首先定义初始化了停止点在规划发布轨迹点中的下标为0
    int stop_index = 0;
    //定义了静态常量速度阈值，速度小于此阈值认为是停止条件之一
    static constexpr double kSpeedThreshold = 1e-3;
    //定义了静态常量加速度常量 前进档时加速度>此阈值且<0认为是停止条件之一
    static constexpr double kForwardAccThreshold = -1e-2;
    //定义了静态常量加速度常量 R档时加速度<此阈值且>0认为是停止条件之一
    static constexpr double kBackwardAccThreshold = 1e-1;
    //定义了静态常量驻车速度，也是判断停车点的一个依据
    static constexpr double kParkingSpeed = 0.1;
 
    //若规划发布的轨迹信息trajectory_message_中档位为D档
    //trajectory_message_是LonController类的成员，是ADCTrajectory message类对象
    //ADCTrajectory message类由planning.proto生成 参见google protobuf库的使用
    if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
      //_size()用于访问message类下repeated数组成员的长度
      //trajectory_point_size()返回的是trajectory_message_指针中的轨迹点个数
      //这里就是stop_index停车点下标从0开始遍历当前规划发布轨迹所有点
      while (stop_index < trajectory_message_->trajectory_point_size()) {
        //定义当前此次循环遍历的轨迹点为轨迹msg中下标为stop_index的那个点
        auto &current_trajectory_point =
      trajectory_message_->trajectory_point(stop_index);
        //若当前遍历的轨迹点速度绝对值 < 速度阈值 且 当前遍历的轨迹点加速度 > 前进加速度阈值 且 当前遍历的轨迹点加速度 < 0
        //若符合这条件则找到了停车点直接break跳出while遍历循环
        if (fabs(current_trajectory_point.v()) < kSpeedThreshold &&
      current_trajectory_point.a() > kForwardAccThreshold &&
      current_trajectory_point.a() < 0.0) {
    break;
        }
        //若此次循环不符合上述停车点的判定条件，则遍历规划发布的轨迹msg中的下一个点
        ++stop_index;
      }
    } else { 
      //若为非前进档的停车点下标搜索，不再赘述
      while (stop_index < trajectory_message_->trajectory_point_size()) {
        auto &current_trajectory_point =
      trajectory_message_->trajectory_point(stop_index);
        if (current_trajectory_point.v() < kSpeedThreshold &&
      current_trajectory_point.a() < kBackwardAccThreshold &&
      current_trajectory_point.a() > 0.0) {
    break;
        }
        ++stop_index;
      }
    }
    //如果到了轨迹msg的最后一个点都没有符合上边符合要求的就将最后一个轨迹点为停止点
    if (stop_index == trajectory_message_->trajectory_point_size()) {
      //c++数组最后一个数下标=数组长度-1，因为第一个是从0开始，所以这里减1
      --stop_index;
      //若最后一个轨迹点速度绝对值 < 驻车速度阈值
      if (fabs(trajectory_message_->trajectory_point(stop_index).v()) <
    kParkingSpeed) {
        //显示打印信息，最后一个点被选为停车点
        ADEBUG << "the last point is selected as parking point";
      } else {
        //否则的话也只是提示终点的速度 > 速度死区而已，停车点仍是选这个，仅仅打印信息不同
        ADEBUG << "the last point found in path and speed > speed_deadzone";
      }
    }
    //将停车点的纵向位置与当前点的纵向位置之差存到debug.path_remain里
    debug->set_path_remain(
        trajectory_message_->trajectory_point(stop_index).path_point().s() -
        debug->current_station());
  }
 
  }  // namespace control
  }  // namespace apollo
//LonController纵向控制器类定义
#pragma once
 
#include <memory>
#include <string>
#include <vector>
 
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/control/common/interpolation_2d.h"
#include "modules/control/common/leadlag_controller.h"
#include "modules/control/common/pid_controller.h"
#include "modules/control/common/trajectory_analyzer.h"
#include "modules/control/controller/controller.h"
 
//LonController类在apollo::control命名空间下定义
namespace apollo {
namespace control {
 
//LonController类，纵向控制器, 来计算 brake/throttle 值.
//实际是继承基类Controller类，Controller类也有必要看一下
//命名规律，类单词首字母大写，实际对象都小写
class LonController : public Controller {
 public:
  //类构造函数
  LonController();
 
  //类析构函数
  virtual ~LonController();
 
  //Init()函数初始化纵向控制器，
  //参数：injector车辆当前状态信息，将其读取到LonController类成员变量injector_里
  //参数：control_conf控制配置信息，将其读取到LonController类成员变量control_conf_里
  common::Status Init(std::shared_ptr<DependencyInjector> injector,
                      const ControlConf *control_conf) override;
 
  //计算 刹车/油门值基于当前车辆的状态和目标轨迹
  //参数：车辆定位信息 指针localization
  //参数：chassis车辆底盘反馈状态信息 指针chassis
  //参数：规划发布的轨迹信息 指针trajectory
  //参数：控制指令 指针cmd，实际上是计算出的控制指令往cmd里填充
  //返回计算状态码
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      control::ControlCommand *cmd) override;
 
  //复位纵向控制器。返回复位的状态码
  common::Status Reset() override;
 
  /**
   * @brief stop longitudinal controller
   */
  //停止纵向控制器，override说明在基类中Stop()接口已经被声明为虚函数，调用的是派生类里的定义
  void Stop() override;
 
  //纵向控制器的名字
  //返回纵向控制器的名字字符串
  //override说明在基类中接口已经被声明为虚函数，调用的是派生类里的定义
  std::string Name() const override;
 
 protected:
  //计算纵向误差函数
  //参数：轨迹信息指针trajectory
  //参数：预览时间preview_time
  //参数：控制周期ts
  //参数：调试信息指针debug用来存放计算的纵向误差信息
  //该函数在control_component.cc中被调用
  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                 const double preview_time, const double ts,
                                 SimpleLongitudinalDebug *debug);
 
  //根据规划发布的轨迹msg寻找轨迹点上接下来的第一个停车点
  //停车点信息存到debug里，这个参数又是用来装结果的
  void GetPathRemain(SimpleLongitudinalDebug *debug);
 
 private:
  //设置Pitch车辆俯仰角
  //参数：lon_controller_conf是纵向控制器配置类对象，该类由modules/control/proto/.proto文件生成
  //从该对象中读取截至频率，控制周期等参数来设置pitch角滤波器
  //pitch角用来进行车辆的坡道补偿，默认坡道补偿是关闭的
  void SetDigitalFilterPitchAngle(const LonControllerConf &lon_controller_conf);
 
  //加载控制标定表函数
  //参数：lon_controller_conf是纵向控制器配置类对象，该类由modules/control/proto/.proto文件生成
  //从纵向控制器配置对象中读取车速-加速度-控制百分数标定表
  void LoadControlCalibrationTable(
      const LonControllerConf &lon_controller_conf);
 
  //设置数字滤波器函数
  //参数：控制周期ts
  //参数：截至频率cutoff_freq
  //参数：数字滤波器类对象digital_filter，前面两项参数就是为了设置这个对象
  void SetDigitalFilter(double ts, double cutoff_freq,
                        common::DigitalFilter *digital_filter);
 
  //关闭日志文件函数
  void CloseLogFile();
 
  //定义成员常量指针localization_，存放的是定位来的信息，初始化为空指针
  const localization::LocalizationEstimate *localization_ = nullptr;
  //定义成员常量指针chassis_，存放的是来自canbus的车辆状态反馈信息，初始化为空指针
  const canbus::Chassis *chassis_ = nullptr;
 
  //定义成员二维插值点指针control_interpolation_，实际就是存放标定表及插值功能
  std::unique_ptr<Interpolation2D> control_interpolation_;
  //定义常量轨迹msg类指针trajectory_message_，是用来存放规划模块发来的轨迹消息
  const planning::ADCTrajectory *trajectory_message_ = nullptr;
  //定义轨迹分析者类对象trajectory_analyzer_，用来实现对各种轨迹信息的解析
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;
 
  //定义成员name_，就是控制器的名称
  std::string name_;
  //定义成员controller_initialized_，实际上就是表明控制器是否被初始化成功
  bool controller_initialized_ = false;
 
  //定义成员上一时刻的加速度
  double previous_acceleration_ = 0.0;
  //定义成员上一时刻的参考加速度
  double previous_acceleration_reference_ = 0.0;
 
  //定义PID控制器类对象speed_pid_controller_，纵向控制器里的速度控制器
  PIDController speed_pid_controller_;
  //定义PID控制器类对象station_pid_controller_，纵向控制器里的位置控制器
  PIDController station_pid_controller_;
 
  //纵向上leadlag控制器默认关闭
  //定义超前滞后控制器类对象 speed_leadlag_controller_，用于速度的控制
  LeadlagController speed_leadlag_controller_;
  //定义超前滞后控制器类对象 station_leadlag_controller_，用于位置的控制
  LeadlagController station_leadlag_controller_;
 
  //定义文件类对象 speed_log_file_用于存放纵向上的日志信息，默认也关闭csv日志
  FILE *speed_log_file_ = nullptr;
 
  //定义数字滤波器类对象digital_filter_pitch_angle_，用于对俯仰角pitch进行滤波
  common::DigitalFilter digital_filter_pitch_angle_;
  //定义常量成员控制配置类对象control_conf_指针，用于存放配置文件中加载进来的控制配置
  const ControlConf *control_conf_ = nullptr;
 
  // 定义成员车辆参数类对象，用于存放车辆的实际尺寸参数等
  common::VehicleParam vehicle_param_;
  //定义成员车辆状态信息类对象injector_,用于存放当前车辆的状态信息
  std::shared_ptr<DependencyInjector> injector_;
  
  //可以看到所有类内要用到的数据基本都在类内再定义了个相对应的数据成员，比如给定位模块来的信息又起了个别名localization_，有利于各个模块间解耦
  //类内所有操作都是假设这些参数已知的情况下，然后只要把这些别名接口与外部模块对齐即可
};
}  // namespace control
}  // namespace apollo
 
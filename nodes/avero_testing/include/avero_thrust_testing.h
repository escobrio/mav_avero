#include <omav_hovery_core/omav_base_client.h>
#include <dynamic_reconfigure/server.h>


class AveroThrustTestingNode {
 public:
  explicit AveroThrustTestingNode(); //constructor
  void start();
  void stop();
  void thrustTesting();

 private:
  omV::OMAVBaseClient<_PWM,_PWM,_PWM> ms_client_;
  std::shared_ptr<omV::ll::MotorInterface<_PWM,_PWM,_PWM>> esc_adapter_;
  ros::Publisher cmd_vel_pub_; // Publisher of the Command values to record

  //Change Values to reasonable for us.
  double maxRampVel_{100.0};                             // RPM/s also needs to be in PWM!
  double startingValue_{1051}, endValue_{1950.0}, step_{10.0};  // RPM we need to convert the RPM to PWM! PWM [1050,1950]
  double stabilizationDelay_{5.0};    
  int holdingTime_;                     // s
};
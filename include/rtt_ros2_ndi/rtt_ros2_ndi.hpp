#include <boost/asio/serial_port.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>

class rtt_ros2_ndi : public RTT::TaskContext{

private:

  // error codes
  enum Errno{ ESUCCESS=0, EFAILURE };

  // structure for a NDI port
  struct Port{
    std::string name;
    std::string serial_number;
    std::string type;
    
    friend std::ostream& operator<<( std::ostream& os, const Port& port ){
      os << "Port handle: " << port.name << std::endl
	 << "Serial number: " << port.serial_number << std::endl
	 << "Type: " << port.type << std::endl;
      return os;
    }

  };

  // map from port ID to port structure
  std::map<std::string, Port> ports;

  // serial port
  boost::asio::io_context io_context;
  boost::asio::serial_port serial;

  // serial port device file
  std::string device_file;

  // recv buffer for serial port
  const static std::size_t BUF_SIZE=512;
  char buffer[BUF_SIZE];

  // ROS service
  RTT::Service::shared_ptr ros;
  // TF2 service
  RTT::Service::shared_ptr tf2;

  // Operation to call tf2 broadcase
  RTT::OperationCaller<void (const geometry_msgs::msg::TransformStamped&)> broadcastTransform;

  // Operation to configure each NDI sensor
  RTT::Operation<bool(std::string,std::string)> configure_sensor;

public:

  rtt_ros2_ndi( const std::string& name );
  ~rtt_ros2_ndi();

  // This adds a sensor to the map
  bool configureSensor( const std::string& port_handle,
			const std::string& name );

  // RTT hooks
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();

  // send/recv serial 
  void send( const std::string& command );
  Errno recv( const std::string& response = std::string() );

};

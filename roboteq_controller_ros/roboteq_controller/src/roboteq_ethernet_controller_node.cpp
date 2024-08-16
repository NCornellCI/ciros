#include "roboteq_controller/roboteq_ethernet_controller_node.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>

// static const std::string tag {"[RoboteQ] "};
static const std::string tag {""};
int controlsock;
int roboteqsock;
int bytes_sent;
struct addrinfo hints, roboteqhints, *controlinfo, *roboteqinfo;

void EtherRoboteqDriver::declare(){
	declare_parameter<std::string>("ethernet_port", "192.168.2.23");
	declare_parameter("closed_loop", false);
	declare_parameter("diff_drive_mode", false);
	declare_parameter("wheel_circumference", 20.0);
	declare_parameter("track_width", 1.5);
	declare_parameter("max_rpm", 200.0);
	declare_parameter<int>("frequency", 30);
	declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
}

void EtherRoboteqDriver::init(){

	RCLCPP_INFO(get_logger(), "Creating");
	get_parameter("frequency", frequency_);

	get_parameter("ethernet_port", ethernet_port_);

	get_parameter("closed_loop", closed_loop_);
	get_parameter("diff_drive_mode", diff_drive_mode_);
	
	get_parameter("wheel_circumference", wheel_circumference_);
	get_parameter("track_width", track_width_);
	get_parameter("max_rpm", max_rpm_);
	get_parameter("cmd_vel_topic", cmd_vel_topic_);

	RCLCPP_INFO_STREAM(this->get_logger(),tag << "cmd_vel:" << cmd_vel_topic_);

	RCLCPP_INFO_STREAM(this->get_logger(),tag << closed_loop_);
	if (closed_loop_){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "In CLOSED-LOOP mode!!!! ethernet port:" << ethernet_port_);
	}
	else{
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "In OPEN-LOOP mode!!!!");
	}

	if (wheel_circumference_ <=0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! wheel_circumference need to be greater than zero.");
	}
	if (track_width_ <=0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! track_width need to be greater than zero.");
	}
	if ( max_rpm_ <= 0.0 ){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! max_rpm need to be greater than zero.");
	}

	if (frequency_ <= 0.0){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! \'frequency\' need to be greater than zero.");
	}

	// Nested params for queries
	// get_parameter();

	auto param_interface = this->get_node_parameters_interface();
	std::map<std::string, rclcpp::ParameterValue> params = param_interface->get_parameter_overrides();

	RCLCPP_INFO_STREAM(this->get_logger(), tag << "queries:" );

	for (auto iter = params.begin(); iter != params.end(); iter++){
		std::size_t pos = iter->first.find("query");
		if (pos != std::string::npos && 
			iter->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
  			std::string topic = iter->first.substr (pos+ 6);    
			auto query = iter->second.to_value_msg().string_value;
			
			queries_[topic] =  query;
			RCLCPP_INFO(this->get_logger(), "%15s : %s",  topic.c_str(), query.c_str() );
		}
	}
}

EtherRoboteqDriver::EtherRoboteqDriver(const rclcpp::NodeOptions &options): Node("roboteq_ethernet_controller", options),
	wheel_circumference_(5.0),
	track_width_(1.5),
	max_rpm_(200.),
	frequency_(50),
	ethernet_port_("192.168.2.23"),
	closed_loop_(0),
	diff_drive_mode_(false),
	cmd_vel_topic_("cmd_vel"){
	
	declare();
	init();

	if (diff_drive_mode_){
		cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
										cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
										std::bind(&EtherRoboteqDriver::cmdVelCallback, this, std::placeholders::_1));
	}
	else{
		cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
										cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
										std::bind(&EtherRoboteqDriver::powerCmdCallback, this, std::placeholders::_1));
	}

	int status;
	
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	memset(&roboteqhints, 0, sizeof roboteqhints);
	roboteqhints.ai_family = AF_INET;
	roboteqhints.ai_socktype = SOCK_STREAM;

	char hostname[32] = "";
	int result = gethostname(hostname, sizeof hostname);

	std::string hostport = "4000";
	std::string roboport = "9761";

	//get this PC IP
	if ((status = getaddrinfo(hostname, hostport.c_str(), &hints, &controlinfo)) != 0){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Unable to open this PC's port");
		rclcpp::shutdown();
	}

	// Get ethernet port data
	if ((status = getaddrinfo(ethernet_port_.c_str(),roboport.c_str(), &roboteqhints, &roboteqinfo)) != 0){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Unable to open port " << ethernet_port_);
		rclcpp::shutdown();
	}

	controlsock = socket(controlinfo->ai_family, controlinfo->ai_socktype, controlinfo->ai_protocol);
	roboteqsock = socket(roboteqinfo->ai_family, roboteqinfo->ai_socktype, roboteqinfo->ai_protocol);

	if((result = bind(controlsock, controlinfo->ai_addr, controlinfo->ai_addrlen)) == -1){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Local Port failed to bind due to: " << strerror(errno));
		rclcpp::shutdown();
	}

	if((result = connect(roboteqsock, roboteqinfo->ai_addr, roboteqinfo->ai_addrlen)) == -1){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Remote Port failed to connect due to: "  << strerror(errno));
		rclcpp::shutdown();
	}

	freeaddrinfo(controlinfo);
	freeaddrinfo(roboteqinfo);

	cmdSetup();

	run();
}


void EtherRoboteqDriver::cmdSetup(){
	// stop motors
	
	std::string msg ="!G 1 0\r";
	int len = strlen(msg.c_str());
	bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

	msg ="!G 2 0\r";
	len = strlen(msg.c_str());
	bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

	msg ="!S 1 0\r";
	len = strlen(msg.c_str());
	bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

	msg ="!S 2 0\r";
	len = strlen(msg.c_str());
	bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

	// // disable echo
	// ser.write("^ECHOF 1\r");
	// ser.flush();

	//RCLCPP_INFO_STREAM(this->get_logger(),tag << "Breakpoint 1");

	// enable watchdog timer (1000 ms) to stop if no connection
	msg ="^RWD 1000\r";
	len = strlen(msg.c_str());
	bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

	//RCLCPP_INFO_STREAM(this->get_logger(), "Breakpoint 2");

	// set motor operating mode (1 for closed-loop speed)
	if (!closed_loop_){
		// open-loop speed mode
		msg ="^MMOD 1 0\r";
		len = strlen(msg.c_str());
		bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

		msg ="^MMOD 2 0\r";
		len = strlen(msg.c_str());
		bytes_sent = send(roboteqsock, msg.c_str(), len, 0);
	}
	else{
		// closed-loop speed mode
		msg ="^MMOD 1 1\r";
		len = strlen(msg.c_str());
		bytes_sent = send(roboteqsock, msg.c_str(), len, 0);

		msg ="^MMOD 2 1\r";
		len = strlen(msg.c_str());
		bytes_sent = send(roboteqsock, msg.c_str(), len, 0);
	}

	//RCLCPP_INFO_STREAM(this->get_logger(), "Breakpoint 3");
	// set encoder counts (ppr)
	
	std::string right_enccmd = "^EPPR 1 1024\r";
	std::string left_enccmd = "^EPPR 2 1024\r";
	
	len = strlen(left_enccmd.c_str());
	bytes_sent = send(roboteqsock, left_enccmd.c_str(), len, 0);

	len = strlen(right_enccmd.c_str());
	bytes_sent = send(roboteqsock, right_enccmd.c_str(), len, 0);
}

void EtherRoboteqDriver::run(){
	// initializeServices();
	std::stringstream ss0, ss1;
	ss0 << "^echof 1_";
	ss1 << "# c_/\"DH?\",\"?\"";

	for (auto item : queries_){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Publish topic: " << item.first);
		query_pub_.push_back(create_publisher<roboteq_interfaces::msg::ChannelValues>(item.first, 100));

		std::string cmd = item.second;
		ss1 << cmd << "_";
	}

	ss1 << "# " << frequency_ << "_";

	const std::string tmp = ss0.str();
	const char* css0 = tmp.c_str();

	const std::string tmp2 = ss1.str();
	const char* css1 = tmp2.c_str();

	int len = strlen(css0);
	send(roboteqsock, css0, len, 0);

	len = strlen(css1);
	send(roboteqsock, css1, len, 0);

    ethernet_read_pub_ = create_publisher<std_msgs::msg::String>("read", rclcpp::SystemDefaultsQoS());

	std::chrono::duration<int, std::milli> dt (1000/frequency_);
	timer_pub_ = create_wall_timer(dt, std::bind(&EtherRoboteqDriver::queryCallback, this) );
}

void EtherRoboteqDriver::powerCmdCallback(const geometry_msgs::msg::Twist &msg){
	std::stringstream cmd_str;
	if (closed_loop_){
		cmd_str << "!S 1"
				<< " " << msg.linear.x << "_"
				<< "!S 2"
				<< " " << msg.angular.z << "_";
	}
	else{
		cmd_str << "!G 1"
				<< " " << msg.linear.x << "_"
				<< "!G 2"
				<< " " << msg.angular.z << "_";
	}

	const std::string tmp3 = cmd_str.str();
	const char* ccmd_str = tmp3.c_str();

	int len = strlen(ccmd_str);
	send(roboteqsock, ccmd_str, len, 0);

	RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9.3f right: %9.3f", msg.linear.x, msg.angular.z);
	// RCLCPP_INFO_STREAM(this->get_logger(),cmd_str.str());
}


void EtherRoboteqDriver::cmdVelCallback(const geometry_msgs::msg::Twist &msg){
	// wheel speed (m/s)
	float right_speed = msg.linear.x + track_width_ * msg.angular.z / 2.0;
	float left_speed  = msg.linear.x - track_width_ * msg.angular.z / 2.0;
	
	// RCLCPP_INFO(this->get_logger(),("[ROBOTEQ] left: %.3f right: %.3f", left_speed, right_speed);
	std::stringstream cmd_str;
	if (!closed_loop_){
		// motor power (scale 0-1000)
		float right_power = right_speed *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
		float left_power  = left_speed  *1000.0 *60.0/ (wheel_circumference_ * max_rpm_);
	
		RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9d right: %9d", (int)left_power, (int)right_power);
		
		cmd_str << "!G 1"
				<< " " << (int)left_power << "_"
				<< "!G 2"
				<< " " << (int)right_power << "_";
	}
	else{
		// motor speed (rpm)
		int32_t right_rpm = right_speed *60.0 / wheel_circumference_;
		int32_t left_rpm  = left_speed  *60.0 / wheel_circumference_;

		RCLCPP_INFO(this->get_logger(),"[ROBOTEQ] left: %9d right: %9d", left_rpm, right_rpm);
		cmd_str << "!S 1"
				<< " " << left_rpm << "_"
				<< "!S 2"
				<< " " << right_rpm << "_";
	}
	const std::string tmp4 = cmd_str.str();
	const char* ccmd_str = tmp4.c_str();

	int len = strlen(ccmd_str);
	send(roboteqsock, ccmd_str, len, 0);
	// RCLCPP_INFO_STREAM(this->get_logger(),cmd_str.str());
}


// bool EtherRoboteqDriver::configService(roboteq_controller::config_srv::Request &request, 
// 									roboteq_controller::config_srv::Response &response){
// 	std::stringstream str;
// 	str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
// 		<< "%\clsav321654987";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = str.str();

// 	RCLCPP_INFO_STREAM(this->get_logger(),tag << response.result);
// 	return true;
// }


// bool EtherRoboteqDriver::commandService(roboteq_controller::command_srv::Request &request, roboteq_controller::command_srv::Response &response)
// {
// 	std::stringstream str;
// 	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = str.str();

// 	RCLCPP_INFO_STREAM(this->get_logger(),tag << response.result);
// 	return true;
// }


// bool EtherRoboteqDriver::maintenanceService(roboteq_controller::maintenance_srv::Request &request, roboteq_controller::maintenance_srv::Response &response)
// {
// 	std::stringstream str;
// 	str << "%" << request.userInput << " "
// 		<< "_";
// 	ser_.write(str.str());
// 	ser_.flush();
// 	response.result = ser_.read(ser_.available());

// 	RCLCPP_INFO_STREAM(this->get_logger(),response.result);
// 	return true;
// }


// void EtherRoboteqDriver::initializeServices(){
// 	configsrv_ 			= nh_.advertiseService("config_service", &EtherRoboteqDriver::configService, this);
// 	commandsrv_ 		= nh_.advertiseService("command_service", &EtherRoboteqDriver::commandService, this);
// 	maintenancesrv_ 	= nh_.advertiseService("maintenance_service", &EtherRoboteqDriver::maintenanceService, this);
// }

void EtherRoboteqDriver::queryCallback(){
	auto current_time = this->now();
	socklen_t size = sizeof(roboteqinfo);
	if (getpeername(roboteqsock, (struct sockaddr*)&roboteqinfo, &size) != -1){
		std_msgs::msg::String result;
		char buf[128] = "";

		recv(roboteqsock, buf, 128 - 1, 0);

		result.data = buf;

		ethernet_read_pub_->publish(result);


		boost::replace_all(result.data, "\r", "");
		boost::replace_all(result.data, "+", "");

		

		std::vector<std::string> fields;
		
		boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
		if (fields.size() < 2){

			RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Empty data:{" << result.data << "}");
		}
		else if (fields.size() >= 2){
			std::vector<std::string> fields_H;
			for (int i = fields.size() - 1; i >= 0; i--){
				if (fields[i][0] == 'H'){
					try{
						fields_H.clear();
						boost::split(fields_H, fields[i], boost::algorithm::is_any_of("?"));
						if ( fields_H.size() >= query_pub_.size() + 1){
							break;
						}
					}
					catch (const std::exception &e){
						std::cerr << e.what() << '\n';
						RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Finding query output in :" << fields[i]);
						continue;
					}
				}
			}

			if (fields_H.size() > 0 && fields_H[0] == "H"){
				for (int i = 0; i < fields_H.size() - 1; ++i){
					std::vector<std::string> sub_fields_H;
					boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
					
					roboteq_interfaces::msg::ChannelValues msg;
					msg.header.stamp = current_time;

					for (int j = 0; j < sub_fields_H.size(); j++){
						try{
							msg.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
						}
						catch (const std::exception &e){
							RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Garbage data on Serial");
							RCLCPP_ERROR_STREAM(this->get_logger(), result.data);
							RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
							std::cerr << e.what() << '\n';
						}
					}
					query_pub_[i]->publish(msg);
				}
			}
		}
		else{
			RCLCPP_WARN_STREAM(this->get_logger(),tag << "Unknown:{" << result.data << "}");
		}
		
	}
	else RCLCPP_ERROR_STREAM(this->get_logger(), strerror(errno));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EtherRoboteqDriver>());
  close(roboteqsock);
  rclcpp::shutdown();
  return 0;
}
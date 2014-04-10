#include <stdio.h>
#include <math.h>
#include <Aria.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>  //for sonar data
#include "nav_msgs/Odometry.h"
#include "rosaria/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <rosaria/RosAriaConfig.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include <sstream>


// imu
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
//


//#define IMU 26
//ArCommands::IMU

class ArCommandsExt : public ArCommands
{
public:
    enum Commands
    {
        IMU = 26
    };
};

class ArIMURobot
{
public:

  AREXPORT ArIMURobot(ArRobot *robot);
  AREXPORT virtual ~ArIMURobot();

  //SeekurJr Operation Manual rev.4, p.35
  //void commandOff(void) { myRobot->comInt(ArCommands::IMU, 0); }
  void commandOff(void) { myRobot->comInt(ArCommandsExt::IMU, 0); }
  void commandOnePacket(void) { myRobot->comInt(ArCommandsExt::IMU, 1); }
  void commandContinuousPackets(void) { myRobot->comInt(ArCommandsExt::IMU, 2); }

  inline double getLinearAccX(void){ return AvgLinearAccX; }
  inline double getLinearAccY(void){ return AvgLinearAccY; }
  inline double getLinearAccZ(void){ return AvgLinearAccZ; }
  inline double getAngularVelX(void) { return AvgAngularVelX; }
  inline double getAngularVelY(void){ return AvgAngularVelY; }
  inline double getAngularVelZ(void){ return AvgAngularVelZ; }
  inline double getTempX(void) { return AvgTempX; }
  inline double getTempY(void){ return AvgTempY; }
  inline double getTempZ(void){ return AvgTempZ; }
  inline double getSeqNum(void){ return seq; }
  inline double getGyroDynamicRange(void){ return gyroDynamicRange; }

protected:
  double AvgLinearAccX;
  double AvgLinearAccY;
  double AvgLinearAccZ;
  double AvgAngularVelX;
  double AvgAngularVelY;
  double AvgAngularVelZ;
  double AvgTempX;
  double AvgTempY;
  double AvgTempZ;

  int numAccelAxes;
  int numGyroAxes;
  int numTemperatureAxes;

  int timeSince; // time since last packet
  int numPerAxis;
  int gyroDynamicRange;

  uint32_t seq; // pkt sequential number

private:
  ArRobot *myRobot;
  ArRetFunctor1C<bool, ArIMURobot, ArRobotPacket*> myPacketHandlerCB;
  bool packetHandler(ArRobotPacket *packet);
};

AREXPORT ArIMURobot::ArIMURobot(ArRobot *robot) :
  myPacketHandlerCB(this, &ArIMURobot::packetHandler)
{
  myRobot = robot;


  AvgLinearAccX =
         AvgLinearAccY =
         AvgLinearAccZ =
         AvgAngularVelX =
         AvgAngularVelY =
         AvgAngularVelZ =
         AvgTempX =
         AvgTempY =
         AvgTempZ = 0.0;

  numAccelAxes =
         numGyroAxes =
         numTemperatureAxes = 0;


  myPacketHandlerCB.setName("ArIMURobot");
  if (myRobot != NULL)
    myRobot->addPacketHandler(&myPacketHandlerCB);
}

AREXPORT ArIMURobot::~ArIMURobot()
{
  if (myRobot != NULL)
    myRobot->remPacketHandler(&myPacketHandlerCB);
}

bool ArIMURobot::packetHandler(ArRobotPacket *packet)
{
    //robot->lock();
    if (packet->getID() != 0x9a) {
      return false;
    }
    timeSince = packet->bufToByte();
    //TimeSinceSIP: Time in ms.
    // Between last SIP sent and sending this packet

    numPerAxis = packet->bufToByte();

    int i;
    double multiplier = 0.0;
    double offset = 0.0;

    numGyroAxes = packet->bufToByte();
    for (i=1; i <= numPerAxis; i++) {
      gyroDynamicRange = packet->bufToByte();
      if (gyroDynamicRange == 3) {
        multiplier = 0.07326;
        offset = 0.0;
      }
      else if (gyroDynamicRange == 2) {
        multiplier = 0.03663;
        offset = 0.0;
      }
      else if (gyroDynamicRange == 1) {
        multiplier = 0.01832;
        offset = 0.0;
      }
      if (numGyroAxes >= 1) {
        AvgAngularVelX = (M_PI/180)*((packet->bufToByte2() * multiplier) + offset); //%10.4f deg/s *(M_PI/180) = rad/sec
      }
      if (numGyroAxes >= 2) {
        AvgAngularVelY = (M_PI/180)*((packet->bufToByte2() * multiplier) + offset); //%10.4f deg/s *(M_PI/180) = rad/sec
      }
      if (numGyroAxes >= 3) {
        AvgAngularVelZ = (M_PI/180)*((packet->bufToByte2() * multiplier) + offset); //%10.4f deg/s *(M_PI/180) = rad/sec
      }
    }

    numAccelAxes = packet->bufToByte();
    for (i=1; i <= numPerAxis; i++) {
      multiplier = (0.333*0.001) * (9.80665); // *(9.80665) => [m/s^2]
      // now, i have no idea about meaning of
      // this number (from imu example): 0.002522
      // according to ADIS16362 specs:
      // Initial Sensitivity ={min:0330,avg:0.333,max:0.336} [milli g/LSB]
      // so: raw_acc_IMUadc_val*(0.333*10e-3)*9.80665 [m/s^2]
      offset = 0.0;
      if (numAccelAxes >= 1) {
        AvgLinearAccX = ((packet->bufToByte2() * multiplier) + offset); //%10.4f mg?
      }
      if (numAccelAxes >= 2) {
        AvgLinearAccY = ((packet->bufToByte2() * multiplier) + offset); //%10.4f mg?
      }
      if (numAccelAxes >= 3) {
        AvgLinearAccZ = ((packet->bufToByte2() * multiplier) + offset); //%10.4f mg?
      }
    }

    // Does temperature used for filtering already?
    numTemperatureAxes = packet->bufToByte();
    for (i=1; i <= numPerAxis; i++) {
      multiplier = 0.1453;
      offset = 25.0;
      if (numTemperatureAxes >= 1) {
        AvgTempX = (packet->bufToByte2() * multiplier) + offset; //%10.4f degC
      }
      if (numTemperatureAxes >= 2) {
        AvgTempY = (packet->bufToByte2() * multiplier) + offset; //%10.4f degC
      }
      if (numTemperatureAxes >= 3) {
        AvgTempZ = (packet->bufToByte2() * multiplier) + offset; //%10.4f degC
      }
    }

  //TODO: It is unknown for me, whether is any correction already introduces in uC or IMU itself.

  //robot->unlock();
  return true;
}


/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library. 

    RosAriaNode will use ARIA to connect to a robot controller (configure via
    ~port parameter), either direct serial connection or over the network.  It 
    runs ARIA's robot communications cycle in a background thread, and
    as part of that cycle (a sensor interpretation task which calls RosAriaNode::publish()),
    it  publishes various topics with newly received robot
    data.  It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.

    For more information about ARIA see
    http://robots.mobilerobots.com/wiki/Aria.

    RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
    information, tutorials and documentation.
*/
class RosAriaNode
{
  public:
    RosAriaNode(ros::NodeHandle n);
    virtual ~RosAriaNode();
    
  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    void sonarConnectCb();
    void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void readParameters();
    void imu_cb();

  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Publisher voltage_pub;

    ros::Publisher linear_acceleration_pub;
    ros::Publisher angular_velocity_pub;
    ros::Publisher temperature_pub;
    ros::Publisher imu_pub;
    ros::Publisher imu_rpy_pub;

    ros::Publisher recharge_state_pub;
    std_msgs::Int8 recharge_state;

    ros::Publisher state_of_charge_pub;

    ros::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    ros::Subscriber cmdvel_sub;

    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::Time veltime;
    ros::Time imutime;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArRobot *robot;
    nav_msgs::Odometry position;
    rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
    //ArRobot::ChargeState batteryCharge;
    ArTCM2 *compass;
    ArIMURobot *adis16326;

    geometry_msgs::Vector3Stamped temperature;
    //sensor_msgs::ImuPtr imu_msg;
    geometry_msgs::Vector3Stamped rpy_msg;

    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    // for imu->base_link transform
    tf::TransformBroadcaster imu_broadcaster;
    tf::Transform imu_transform;
    tf::Quaternion orientation;
    geometry_msgs::Quaternion orientation_msg;


    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;

    std::string frame_id_sonar;
    std::string frame_id_imu_link;
    std::string frame_id_mag_link;


    //Imu support
    bool use_imu;

    //Magnetometer support
    bool use_mag;

    //Sonar support
    bool use_sonar;  // enable and publish sonars

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    
    // Robot Parameters
    int TicksMM, DriftFactor, RevCount;  // Odometry Calibration Settings
    
    // dynamic_reconfigure
    dynamic_reconfigure::Server<rosaria::RosAriaConfig> dynamic_reconfigure_server;
};

void RosAriaNode::readParameters()
{
  // Robot Parameters  
  robot->lock();
  ros::NodeHandle n_("~");
  if (n_.hasParam("TicksMM"))
  {
    n_.getParam( "TicksMM", TicksMM);
    if(TicksMM != 0)
    {
      ROS_INFO("Setting TicksMM from ROS Parameter: %d", TicksMM);
      robot->comInt(93, TicksMM);
    }
  }
  
  if (n_.hasParam("DriftFactor"))
  {
    n_.getParam( "DriftFactor", DriftFactor);
    if(DriftFactor != -99999) 
    {
      ROS_INFO("Setting DriftFactor from ROS Parameter: %d", DriftFactor);
      robot->comInt(89, DriftFactor);
    }
  }
  
  if (n_.hasParam("RevCount"))
  {
    n_.getParam( "RevCount", RevCount);
    if(RevCount != 0)
    {
      ROS_INFO("Setting RevCount from ROS Parameter: %d", RevCount);
      robot->comInt(88, RevCount);
    }
  }
  robot->unlock();
}

void RosAriaNode::dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level)
{
  //
  // Odometry Settings
  //
  robot->lock();
  if(TicksMM != config.TicksMM && config.TicksMM != 0)
  {
    ROS_INFO("Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, config.TicksMM);
    TicksMM = config.TicksMM;
    robot->comInt(93, TicksMM);
  }
  
  if(DriftFactor != config.DriftFactor && config.DriftFactor != -99999) 
  {
    ROS_INFO("Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, config.DriftFactor);
    DriftFactor = config.DriftFactor;
    robot->comInt(89, DriftFactor);
  }
  
  if(RevCount != config.RevCount && config.RevCount != 0)
  {
    ROS_INFO("Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, config.RevCount);
    RevCount = config.RevCount;
    robot->comInt(88, RevCount);
  }
  
  //
  // Acceleration Parameters
  //
  int value;
  value = config.trans_accel * 1000;
  if(value != robot->getTransAccel() && value > 0)
  {
    ROS_INFO("Setting TransAccel from Dynamic Reconfigure: %d", value);
    robot->setTransAccel(value);
  }
  
  value = config.trans_decel * 1000;
  if(value != robot->getTransDecel() && value > 0)
  {
    ROS_INFO("Setting TransDecel from Dynamic Reconfigure: %d", value);
    robot->setTransDecel(value);
  } 
  
  value = config.lat_accel * 1000;
  if(value != robot->getLatAccel() && value > 0)
  {
    ROS_INFO("Setting LatAccel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatAccel() > 0 )
      robot->setLatAccel(value);
  }
  
  value = config.lat_decel * 1000;
  if(value != robot->getLatDecel() && value > 0)
  {
    ROS_INFO("Setting LatDecel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatDecel() > 0 )
      robot->setLatDecel(value);
  }
  
  value = config.rot_accel * 180/M_PI;
  if(value != robot->getRotAccel() && value > 0)
  {
    ROS_INFO("Setting RotAccel from Dynamic Reconfigure: %d", value);
    robot->setRotAccel(value);
  }
  
  value = config.rot_decel * 180/M_PI;
  if(value != robot->getRotDecel() && value > 0)
  {
    ROS_INFO("Setting RotDecel from Dynamic Reconfigure: %d", value);
    robot->setRotDecel(value);
  } 
  robot->unlock();
}

void RosAriaNode::sonarConnectCb()
{
  robot->lock();
  if (sonar_pub.getNumSubscribers() == 0)
  {
    robot->disableSonar();
    use_sonar = false;
  }
  else
  {
    robot->enableSonar();
    use_sonar = true;
  }
  robot->unlock();
}

RosAriaNode::RosAriaNode(ros::NodeHandle nh) : 
    myPublishCB(this, &RosAriaNode::publish), serial_port(""), serial_baud(0), use_sonar(false), use_imu(true), use_mag(false),
  TicksMM(0), RevCount(0), DriftFactor(-99999)
{
  // read in config options
  n = nh;

  // serial port
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "RosAria: using port: [%s]", serial_port.c_str() );

  n.param("baud", serial_baud, 0);
  if(serial_baud != 0)
  ROS_INFO("RosAria: using serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  n.param( "debug_aria", debug_aria, false );
  n.param( "aria_log_filename", aria_log_filename, std::string("Aria.log") );

  // Figure out what frame_id's to use. if a tf_prefix param is specified,
  // it will be added to the beginning of the frame_ids.
  //
  // e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
  // roslaunch files)
  // will result in the frame_ids being set to /MyRobot/odom etc,
  // rather than /odom. This is useful for Multi Robot Systems.
  // See ROS Wiki for further details.
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odom");
  frame_id_base_link = tf::resolve(tf_prefix, "base_link");
  frame_id_bumper = tf::resolve(tf_prefix, "bumpers_frame");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");
  frame_id_imu_link = tf::resolve(tf_prefix, "imu_frame");
  frame_id_mag_link = tf::resolve(tf_prefix, "mag_frame");

  // advertise services for data topics
  // second argument to advertise() is queue size.
  // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
  // subscribers when they subscribe).
  // See ros::NodeHandle API docs.
  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = n.advertise<rosaria::BumperState>("bumper_state",1000);
  sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar", 50,
    boost::bind(&RosAriaNode::sonarConnectCb, this),
    boost::bind(&RosAriaNode::sonarConnectCb, this));

  voltage_pub = n.advertise<std_msgs::Float64>("battery_voltage", 1000);
  recharge_state_pub = n.advertise<std_msgs::Int8>("battery_recharge_state", 5, true /*latch*/ );
  recharge_state.data = -2;
  state_of_charge_pub = n.advertise<std_msgs::Float32>("battery_state_of_charge", 100);

  motors_state_pub = n.advertise<std_msgs::Bool>("motors_state", 5, true /*latch*/ );
  motors_state.data = false;
  published_motors_state = false;

  temperature_pub =  n.advertise<geometry_msgs::Vector3Stamped> ("imu_raw/temperature", 1000);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  imu_rpy_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu_raw/rpy_angles", 100);

  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
    boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));

  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosAriaNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosAriaNode::disable_motors_cb, this);
  
  veltime = ros::Time::now();
}

RosAriaNode::~RosAriaNode()
{
  // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

int RosAriaNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  
  //this_robot = robot; // take a reference for imu handler
  
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort"); // pass robot's serial port to Aria
    args->add(serial_port.c_str());
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud");
    char tmp[100];
    snprintf(tmp, 100, "%d", serial_baud);
    args->add(tmp);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }


  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    ROS_ERROR("RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device.)");
    return 1;
  }

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    ROS_ERROR("RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  readParameters();

  // start dynamic_reconfigure server
  rosaria::RosAriaConfig dynConf_max;
  rosaria::RosAriaConfig dynConf_min;
  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

  dynConf_min.trans_accel = 0;
  dynConf_min.trans_decel = 0;
  dynConf_min.lat_accel = 0;
  dynConf_min.lat_decel = 0;
  dynConf_min.rot_accel = 0;
  dynConf_min.rot_decel = 0;
  
  dynConf_min.TicksMM     = 0;
  dynConf_max.TicksMM     = 200;
  dynConf_min.DriftFactor = -99999;
  dynConf_max.DriftFactor = 32767;
  dynConf_min.RevCount    = 0;
  dynConf_max.RevCount    = 65535;

  dynamic_reconfigure_server.setConfigMax(dynConf_max);
  dynamic_reconfigure_server.setConfigMin(dynConf_min);
  
  
  rosaria::RosAriaConfig dynConf_default;
  dynConf_default.trans_accel = robot->getTransAccel() / 1000;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000;
  dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
  dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

  dynConf_default.TicksMM     = 0;
  dynConf_default.DriftFactor = -99999;
  dynConf_default.RevCount    = 0;
  
  dynamic_reconfigure_server.setConfigDefault(dynConf_default);
  
  dynamic_reconfigure_server.setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));

  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());
  

  // Run ArRobot background processing thread
  robot->runAsync(true);


  // Prepare transform
  // TODO: Should this be here??
  imu_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );

  if (use_imu) {
    // TODO: Check if sim (no imu support)
    adis16326 = new ArIMURobot(robot);
    adis16326->commandContinuousPackets();
  }

  if (use_mag) {
    ArCompassConnector compassConnector(argparser);
    // Create and connect to the compass if the robot has one.
    compass = compassConnector.create(robot);
    if(compass && !compass->blockingConnect()) {
        compass = NULL;
    }
    compass->commandContinuousPackets();
  }

  return 0;
}

void RosAriaNode::imu_cb()
{
    float a_x, a_y, a_z, omega_x, omega_y, omega_z;
    float pitchAcc, rollAcc;
    float  pitch, roll, yaw;

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

    const float GYROSCOPE_SENSITIVITY = 1.0;

    ros::Time time;
    double dt;
    uint32_t seq;

    time = ros::Time::now();

    dt = (time - imutime).toSec();
    imutime = time;
    seq = adis16326->getSeqNum();

    temperature.vector.x = adis16326->getTempX();
    temperature.vector.y = adis16326->getTempY();
    temperature.vector.z = adis16326->getTempZ();
    temperature.header.seq = seq;
    temperature.header.frame_id = frame_id_imu_link;
    temperature.header.stamp = time;


    a_x = adis16326->getLinearAccX();
    a_y = adis16326->getLinearAccY();
    a_z = adis16326->getLinearAccZ();
    omega_x = adis16326->getAngularVelX();
    omega_y = adis16326->getAngularVelY();
    omega_z = adis16326->getAngularVelZ();

    // In the " (pitch-roll-yaw) convention,"
    // theta is pitch, psi is roll, and  phi is yaw.
    // http://mathworld.wolfram.com/EulerAngles.html
    pitch += ((float)omega_x / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    roll -= ((float)omega_y / GYROSCOPE_SENSITIVITY) * dt; // Angle around the Y-axis

    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2(a_x, ( sqrt(pow(a_y,2.0) + pow(a_z,2.0)) )) * 180.0 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2(a_y, (sqrt(pow(a_x,2.0) + pow(a_z,2.0)) )) * 180.0 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;

    yaw =  ((float)omega_z / GYROSCOPE_SENSITIVITY) * dt;

    rpy_msg.vector.x = roll;
    rpy_msg.vector.y = pitch;
    rpy_msg.vector.z = yaw;
    rpy_msg.header.seq = seq;
    rpy_msg.header.frame_id = frame_id_imu_link;
    rpy_msg.header.stamp = time;

    orientation.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(orientation, orientation_msg);

    //build imu_msg
    imu_msg->header.frame_id = frame_id_imu_link;
    imu_msg->header.seq = seq; //imu_raw.time_usec / 1000;
    imu_msg->header.stamp = time;
    imu_msg->orientation = orientation_msg;

    imu_msg->angular_velocity.x = omega_x;
    imu_msg->angular_velocity.y = omega_y;
    imu_msg->angular_velocity.z = omega_z;

    imu_msg->linear_acceleration.x = a_x;
    imu_msg->linear_acceleration.y = a_y;
    imu_msg->linear_acceleration.z = a_z;

    //imu_msg.orientation_covariance
    for (sensor_msgs::Imu::_orientation_covariance_type::iterator it = imu_msg->orientation_covariance.begin();
        it != imu_msg->orientation_covariance.end(); ++it)
      *it = 0;
    //imu_msg.angular_velocity_covariance
    for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator it =
        imu_msg->angular_velocity_covariance.begin(); it != imu_msg->angular_velocity_covariance.end(); ++it)
      *it = 0;
    //imu_msg.linear_acceleration_covariance
    for (sensor_msgs::Imu::_linear_acceleration_covariance_type::iterator it =
        imu_msg->linear_acceleration_covariance.begin(); it != imu_msg->linear_acceleration_covariance.end();
        ++it)
      *it = 0;

    imu_transform.setRotation(orientation);
    imu_broadcaster.sendTransform(tf::StampedTransform(imu_transform, time, frame_id_base_link, frame_id_imu_link));

    temperature_pub.publish(temperature);
    imu_rpy_pub.publish(rpy_msg);
    imu_pub.publish(imu_msg);
}

void RosAriaNode::spin()
{
  ros::spin();
}


void RosAriaNode::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  // set covariance
  // for twist
  for (nav_msgs::Odometry::_twist_type::_covariance_type::iterator it = position.twist.covariance.begin();
      it != position.twist.covariance.end(); ++it)
    *it = 0;
  // for pose
  for (auto it = position.pose.covariance.begin(); it != position.pose.covariance.end(); ++it)
    *it = 0;

  // TODO: set some real values here
  position.pose.covariance =
  {{
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
   }};

  position.pose.covariance =
  {{
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
   }};

  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);
  ROS_DEBUG("RosAria: publish vel: (time %f) linear x: %f, y: %f; angular z: %f", 
    position.header.stamp.toSec(), 
    (double) position.twist.twist.linear.x,
    (double) position.twist.twist.linear.y,
    (double) position.twist.twist.angular.z
  );

  // publishing transform odom->base_link
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);
  
  odom_broadcaster.sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  ROS_DEBUG("RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_DEBUG("RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    ROS_INFO("RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	ROS_INFO("RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (use_sonar) {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
    

    // Log debugging info
    std::stringstream sonar_debug_info;
    sonar_debug_info << "Sonar readings: ";
    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        ROS_WARN("RosAria: Did not receive a sonar reading.");
        continue;
      }
      
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
      
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    ROS_DEBUG_STREAM(sonar_debug_info.str());
    
    sonar_pub.publish(cloud);
  }

  if (use_imu) {
      imu_cb();
  }

  if (use_mag) {
      /*
      // not tested as i have no magnetic field sensor
      tcm2.id = compass->getCompass();
      tcm2.roll = compass->getRoll();
      tcm2.pitch = compass->getPitch();
      tcm2.magnetic.vector.x = compass->getXMagnetic();
      tcm2.magnetic.vector.y = compass->getYMagnetic();
      tcm2.magnetic.vector.z = compass->getZMagnetic();
      tcm2.temperature = compass->getTemperature();
      tcm2.error = compass->getError();
      tcm2.calibration.vector.H = compass->getCalibrationH();
      tcm2.calibration.vector.V = compass->getCalibrationV();
      tcm2.calibration.vector.M = compass->getCalibrationM();
      tcm2.pac_count = compass->getPacCount();
      tcm2.header.seq = seq;
      tcm2.header.frame_id = frame_id_mag_link;
      tcm2.header.stamp = ts;

      tcm2_pub.publish(tcm2);
      
      ROS_INFO("\r%6.1f %5.1f %5.1f %6.2f %6.2f %6.2f %6.1f 0x%08x %4.0f %4.0f %6.2f %3d",
             compass->getCompass(), compass->getPitch(),
             compass->getRoll(), compass->getXMagnetic(),
             compass->getYMagnetic(), compass->getZMagnetic(),
             compass->getTemperature(), compass->getError(),
             compass->getCalibrationH(), compass->getCalibrationV(),
             compass->getCalibrationM(), compass->getPacCount());
      */
  }

}


bool RosAriaNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN("RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}


bool RosAriaNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}


void
RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
  ROS_DEBUG( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  ROS_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
}

int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
  
}

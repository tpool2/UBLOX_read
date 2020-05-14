#include <iostream>
#include <fstream>
#include <signal.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;


#include <UBLOX/ublox_ros.h>

namespace ublox_ros
{

UBLOX_ROS::UBLOX_ROS() :
    nh_(), nh_private_("~")
{

    // Connect ROS topics
    pvt_pub_ = nh_.advertise<ublox::PositionVelocityTime>("PosVelTime", 10);
    relpos_pub_ = nh_.advertise<ublox::RelPos>("RelPos", 10);
    relposflag_pub_ = nh_.advertise<ublox::RelPosFlags>("RelPosFlags", 10);
    ecef_pub_ = nh_.advertise<ublox::PosVelEcef>("PosVelEcef", 10);
    survey_status_pub_ = nh_.advertise<ublox::SurveyStatus>("SurveyStatus", 10);
    eph_pub_ = nh_.advertise<ublox::Ephemeris>("Ephemeris", 10);
    geph_pub_ = nh_.advertise<ublox::GlonassEphemeris>("GlonassEphemeris", 10);
    obs_pub_ = nh_.advertise<ublox::ObsVec>("Obs", 10);
    base_ecef_pub_ = nh_.advertise<ublox::PosVelEcef>("base/PosVelEcef", 10);
    base_pvt_pub_ = nh_.advertise<ublox::PositionVelocityTime>("base/PosVelTime", 10);
    // nav_sat_fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("NavSatFix");
    // nav_sat_status_pub_ = nh_.advertise<sensor_msgs::NavSatStatus>("NavSatStatus");

    // Connect ROS services
    cfg_val_get_ = nh_.advertiseService("CfgValGet", &UBLOX_ROS::cfgValGet, this);
    cfg_val_get_all_ = nh_.advertiseService("CfgValGetAll", &UBLOX_ROS::cfgValGetAll, this);
    cfg_val_del_ = nh_.advertiseService("CfgValDel", &UBLOX_ROS::cfgValDel, this);
    cfg_val_set_ = nh_.advertiseService("CfgValSet", &UBLOX_ROS::cfgValSet, this);
    cfg_reset_ = nh_.advertiseService("CfgReset", &UBLOX_ROS::cfgReset, this);
    init_module_ = nh_.advertiseService("InitModule", &UBLOX_ROS::initModule, this);

    //Get the serial port
    serial_port_ = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");
    log_filename_ = nh_private_.param<std::string>("log_filename", "");
    message_rate_ = nh_private_.param<int>("message_rate", 10); //rate at which GNSS measurements are takens in hz
    rover_quantity_ = nh_private_.param<int>("rover_quantity", 0);
    chain_level_ = nh_private_.param<int>("chain_level", 0x00);    //Get chain_level. 0 is stationary base. 1 to n-1 is
    
    // Get Constallation settings
    gps_ = nh_private_.param<int>("GPS", 1); //GPS
    glonas_ = nh_private_.param<int>("GLONAS", 0); //GLONAS
    beidou_ = nh_private_.param<int>("BEIDOU", 0); //BEIDOU
    galileo_ = nh_private_.param<int>("GALILEO", 1); //GALILEO
    std::cerr << "message_rate = " << message_rate_ << "\n";
    std::cerr << "rover_quantity = " << rover_quantity_ << "\n";
    std::cerr << "chain_level = " << chain_level_ << "\n";
    std::cerr << "gps = " << gps_ << "\n";
    std::cerr << "glonas = " << glonas_ << "\n";
    std::cerr << "beidou = " << beidou_ << "\n";
    std::cerr << "galileo = " << galileo_ << "\n";

    // create the parser
    ublox_ = new ublox::UBLOX(serial_port_, message_rate_);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    if(nh_private_.param<bool>("debug", false))
    {
        std::cerr<<"DEBUG MODE\n";
    }
    else if (chain_level_ == ublox::UBLOX::BASE)
    {
        initBase();
    }
    // Rover(1 local_host 1 local_port 1 base_host 1 base_port)
    else if (rover_quantity_ == 0)
    {
        initRover();
    }
    // Brover(1 base_host 1 base_port n local_host n local_port n rover_host n rover_port)
    else if (rover_quantity_>=0) 
    {
        initBrover();
    }

    // Check if there is a arrow
    if (nh_private_.hasParam("arrowbase") && nh_private_.hasParam("arrowtip")) {

      // If there is an arrow , then we need to subscribe to the base
      std::string arrowbase = nh_private_.param<std::string>("arrowbase", "/brover");
      // and tip of the arrow for /RelPos
      std::string arrowtip = nh_private_.param<std::string>("arrowtip", "/rover");

      // Call the first subscriber
      sub1 = nh_.subscribe(arrowbase+"/RelPos", 10, &UBLOX_ROS::cb_rov1, this);

      // Call the second subscriber
      sub2 = nh_.subscribe(arrowtip+"/RelPos", 10, &UBLOX_ROS::cb_rov2, this);

      // Make the arrow flag true. This flag is used in the relposCB function in
      // in order to determine if the vector math function needs to be called or
      // not.
      arrow_flag = true;
    }

    // connect callbacks
    createCallback(ublox::CLASS_NAV, ublox::NAV_RELPOSNED, &UBLOX_ROS::relposCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_POSECEF, &UBLOX_ROS::posECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_VELECEF, &UBLOX_ROS::velECEFCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_SVIN, &UBLOX_ROS::svinCB, this);
    createCallback(ublox::CLASS_RXM, ublox::RXM_RAWX, &UBLOX_ROS::obsCB, this);
    createCallback(ublox::CLASS_NAV, ublox::NAV_PVT, &UBLOX_ROS::pvtCB, this);

    if (!log_filename_.empty())
    {
        ublox_->initLogFile(log_filename_);
        //ublox_->readFile(log_filename_);
    }
}

UBLOX_ROS::~UBLOX_ROS()
{
    if (ublox_)
        delete ublox_;
}

bool UBLOX_ROS::evalF9PID(uint8_t f9pID)
{
    switch(f9pID)
    {
        case 0:
            ecef_ptr_= &ecef_msg_;
            ecef_pub_ptr_ = &ecef_pub_;
            pvt_ptr_ = &pvt_msg_;
            pvt_pub_ptr_ = &pvt_pub_;
            ecef_pos_tow_ptr_ = &ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &ecef_vel_tow_;
            pvt_tow_ptr_ = &pvt_tow_;
            return true;
            break;
        case 1:
            ecef_ptr_= &base_ecef_msg_;
            ecef_pub_ptr_ = &base_ecef_pub_;
            pvt_ptr_ = &base_pvt_msg_;
            pvt_pub_ptr_ = &base_pvt_pub_;
            ecef_pos_tow_ptr_ = &base_ecef_pos_tow_;
            ecef_vel_tow_ptr_ = &base_ecef_vel_tow_;
            pvt_tow_ptr_ = &base_pvt_tow_;
            return true;
            break;
        default:
            return false;
            break;
    }
}

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ublox_ros");

    ublox_ros::UBLOX_ROS Thing;

    ros::spin();
}

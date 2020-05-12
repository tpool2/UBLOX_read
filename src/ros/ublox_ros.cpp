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
    cfg_val_get = nh_.advertiseService("CfgValGet", &UBLOX_ROS::cfgValGet, this);
    cfg_val_del_ = nh_.advertiseService("CfgValDel", &UBLOX_ROS::cfgValDel, this);
    cfg_val_set_ = nh_.advertiseService("CfgValSet", &UBLOX_ROS::cfgValSet, this);
    cfg_reset_ = nh_.advertiseService("CfgReset", &UBLOX_ROS::cfgReset, this);

    //Get the serial port
    std::string serial_port = nh_private_.param<std::string>("serial_port", "/dev/ttyACM0");
    std::string log_filename = nh_private_.param<std::string>("log_filename", "");
    int message_rate = nh_private_.param<int>("message_rate", 10); //rate at which GNSS measurements are takens in hz
    int rover_quantity = nh_private_.param<int>("rover_quantity", 0);
    int chain_level = nh_private_.param<int>("chain_level", 0x00);    //Get chain_level. 0 is stationary base. 1 to n-1 is
    // Get Constallation settings
    int gps = nh_private_.param<int>("GPS", 1); //GPS
    int glonas = nh_private_.param<int>("GLONAS", 0); //GLONAS
    int beidou = nh_private_.param<int>("BEIDOU", 0); //BEIDOU
    int galileo = nh_private_.param<int>("GALILEO", 1); //GALILEO
    std::cerr << "message_rate = " << message_rate << "\n";
    std::cerr << "rover_quantity = " << rover_quantity << "\n";
    std::cerr << "chain_level = " << chain_level << "\n";
    std::cerr << "gps = " << gps << "\n";
    std::cerr << "glonas = " << glonas << "\n";
    std::cerr << "beidou = " << beidou << "\n";
    std::cerr << "galileo = " << galileo << "\n";

    // create the parser
    ublox_ = new ublox::UBLOX(serial_port, message_rate);

    // set up RTK
    // Base (n local_host n local_port, n rover_host, n rover_port)
    if (chain_level == ublox::UBLOX::BASE){
        std::cerr<<"Initializing Base\n";

        //Get base parameters
        std::string base_type = nh_private_.param<std::string>("base_type", "moving");
        int surveytime = nh_private_.param<int>("Surveytime", 120); //Stationary base survey time
        int surveyacc = nh_private_.param<int>("Surveyacc", 500000);  //Stationary base accuracy
        std::cerr << "base_type = " << base_type << "\n";
        std::cerr << "surveytime = " << surveytime << "\n";
        std::cerr << "surveyacc = " << surveyacc << "\n";


        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[std::max(1, rover_quantity)];
        uint16_t* local_port = new uint16_t[std::max(1, rover_quantity)];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[std::max(1, rover_quantity)];
        uint16_t* rover_port = new uint16_t[std::max(1, rover_quantity)];
        
        //Account for the case when no numbers are used for the first rover.
        uint8_t j = 0;
        if(nh_private_.hasParam("local_host")) {
            //The first local host corresponds to the first rover.
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            
            //First rover
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);

            //Let the program know that we have inputted the first rover.
            j=1;
        }

        for(int i=1+j; i <= rover_quantity; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
        }

        ublox_->initBase(local_host, local_port, rover_host, rover_port,
          base_type, rover_quantity, gps, glonas, beidou, galileo, surveytime,
          surveyacc);
    }
    // Rover(1 local_host 1 local_port 1 base_host 1 base_port)
    else if (rover_quantity == 0){

        std::cerr<<"Initializing Rover\n";

        //Initialize local arrays to contain parameters from xml file
        std::string* local_host = new std::string[1];
        uint16_t* local_port = new uint16_t[1];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        if(nh_private_.hasParam("local_host")) {
            std::string test = nh_private_.param<std::string>("local_host", "localhost");
            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
            base_port[0] = nh_private_.param<int>("base_port", 16145);
        }
        else {
          local_host[0] = nh_private_.param<std::string>("local_host1", "localhost");
          local_port[0] = nh_private_.param<int>("local_port1", 16140);
          base_host[0] = nh_private_.param<std::string>("base_host1", "localhost");
          base_port[0] = nh_private_.param<int>("base_port1", 16145);
        }

        ublox_->initRover(local_host[0], local_port[0], base_host[0], base_port[0]);
    }
    // Brover(1 base_host 1 base_port n local_host n local_port n rover_host n rover_port)
    else if (rover_quantity>=0) {
        std::cerr<<"Initializing Brover\n";

        // Initialize local arrays to contain parameters from xml file
        // The first local_host and local_port correspond to the base.
        std::string* local_host = new std::string[rover_quantity+1];
        uint16_t* local_port = new uint16_t[rover_quantity+1];

        //Initialize rover arrays to contain parameters from xml file
        std::string* rover_host = new std::string[rover_quantity];
        uint16_t* rover_port = new uint16_t[rover_quantity];

        //Initialize base arrays to contain parameters from xml file
        std::string* base_host = new std::string[1];
        uint16_t* base_port = new uint16_t[1];

        // Fill base arrays with their single values
        base_host[0] = nh_private_.param<std::string>("base_host", "localhost");
        base_port[0] = nh_private_.param<int>("base_port", 16140);

        // Get Constallation settings
        uint32_t constellation [6];
        uint8_t gps = nh_.param<int>("GPS", 1);
        uint8_t glonas = nh_.param<int>("GLONAS", 0);
        uint8_t beidou = nh_.param<int>("BEIDOU", 0);
        uint8_t galileo = nh_.param<int>("GALILEO", 1);

        uint8_t j = 0;
        if(nh_private_.hasParam("local_host")) {

            local_host[0] = nh_private_.param<std::string>("local_host", "localhost");
            local_port[0] = nh_private_.param<int>("local_port", 16140);
            rover_host[0] = nh_private_.param<std::string>("rover_host", "localhost");
            rover_port[0] = nh_private_.param<int>("rover_port", 16145);
            j=1;
        }

        //Input parameters from xml file into respective arrays
        for(int i=1+j; i <= rover_quantity; i++) {
            local_host[i-1] = nh_private_.param<std::string>("local_host"+std::to_string(i), "localhost");
            local_port[i-1] = nh_private_.param<int>("local_port"+std::to_string(i), 16140);
            rover_host[i-1] = nh_private_.param<std::string>("rover_host"+std::to_string(i), "localhost");
            rover_port[i-1] = nh_private_.param<int>("rover_port"+std::to_string(i), 16145);
            j = i;
        }

        // Add in extra local host values.
        local_host[j] = nh_private_.param<std::string>("local_host"+std::to_string(j+1), "localhost");
        local_port[j] = nh_private_.param<int>("local_port"+std::to_string(j+1), 16140);

        //Determine whether the brover is moving or stationary?
        std::string base_type = "moving";

        // Initiate the Brover
        ublox_->initBrover(local_host, local_port, base_host, base_port,
           rover_host, rover_port, base_type, rover_quantity, gps,
            glonas, beidou, galileo);

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

    if (!log_filename.empty())
    {
        ublox_->initLogFile(log_filename);
        //ublox_->readFile(log_filename);
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

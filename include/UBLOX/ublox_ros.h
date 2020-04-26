#ifndef UBLOX_ROS_H
#define UBLOX_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "UBLOX/ublox.h"

#include "ublox/PosVelEcef.h"
#include "ublox/PositionVelocityTime.h"
#include "ublox/RelPos.h"
#include "ublox/SurveyStatus.h"
#include "ublox/Ephemeris.h"
#include "ublox/GlonassEphemeris.h"
#include "ublox/Observation.h"
#include "ublox/ObsVec.h"

#include "ublox/CfgValGet.h"
#include "ublox/CfgValDel.h"
#include "ublox/CfgValSet.h"

namespace ublox_ros
{

class UBLOX_ROS
{
public:
    UBLOX_ROS();
    ~UBLOX_ROS();

private:
    ublox::UBLOX* ublox_ = nullptr;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pvt_pub_;
    ros::Publisher survey_status_pub_;
    ros::Publisher relpos_pub_;
    ros::Publisher ecef_pub_;
    ros::Publisher nav_sat_fix_pub_;
    ros::Publisher nav_sat_status_pub_;
    ros::Publisher eph_pub_;
    ros::Publisher geph_pub_;
    ros::Publisher obs_pub_;

    void pvtCB(const ublox::UBX_message_t &ubxmsg);
    void relposCB(const ublox::UBX_message_t &ubx_msg);
    void posECEFCB(const ublox::UBX_message_t &ubx_msg);
    void velECEFCB(const ublox::UBX_message_t &ubx_msg);
    void svinCB(const ublox::UBX_message_t &ubx_msg);

    void obsCB(const ublox::UBX_message_t &ubx_msg);
    void ephCB(const Ephemeris& eph);
    void gephCB(const GlonassEphemeris& eph);

    bool cfgValGet(ublox::CfgValGet::Request &req, ublox::CfgValGet::Response &res);
    bool cfgValDel(ublox::CfgValDel::Request &req, ublox::CfgValDel::Response &res);
    bool cfgValSet(ublox::CfgValSet::Request &req, ublox::CfgValSet::Response &res);
    ros::ServiceServer cfg_val_get;
    ros::ServiceServer cfg_val_del_;
    ros::ServiceServer cfg_val_set_;

    uint32_t pos_tow_;
    uint32_t vel_tow_;
    uint32_t pvt_tow_;
    uint32_t pvt_week_;
    // int message_rate;

    double ned_1[3];
    double ned_2[3];
    bool arrow_flag = false;
    double arrow[7];

    ros::Subscriber sub1;
    ros::Subscriber sub2;

    ublox::PosVelEcef ecef_msg_;
    void cb_rov1(const ublox::RelPos &msg);
    void cb_rov2(const ublox::RelPos &msg);
    

    template<class M> void createCallback(uint8_t cls, uint8_t type, 
        void(ublox_ros::UBLOX_ROS::*fp)(const M &msg), ublox_ros::UBLOX_ROS *obj, uint8_t f9pID=0)
    {
        do
        {
            auto trampoline = [obj, fp](uint8_t _class, uint8_t _type, const ublox::UBX_message_t &ubx_msg)
            {
                (obj->*fp)(ubx_msg);
            };

            this->ublox_->registerUBXCallback(cls, type, trampoline);
        } while (0);
    };
    
};

}

#endif

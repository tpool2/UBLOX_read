#include <UBLOX/ublox_log.h>

namespace ublox
{
	void UBLOX_LOG::pvtCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID)
	{
		std::cerr<<"PVT Callback, F9PID: "<<uint16_t(f9PID)<<std::endl;

		ublox::NAV_PVT_t msg = ubx_msg.NAV_PVT;
		
		ofs_pvt_<<std::endl;
		ofs_pvt_<<msg.iTOW<<",";
		ofs_pvt_<<msg.year<<",";
		ofs_pvt_<<uint16_t(msg.month)<<",";
		ofs_pvt_<<uint16_t(msg.day)<<",";
		ofs_pvt_<<uint16_t(msg.hour)<<",";
		ofs_pvt_<<uint16_t(msg.min)<<",";
		ofs_pvt_<<uint16_t(msg.sec)<<",";
		ofs_pvt_<<uint16_t(msg.valid)<<",";
		ofs_pvt_<<msg.tAcc<<",";
		ofs_pvt_<<msg.nano<<",";
		ofs_pvt_<<uint16_t(msg.fixType)<<",";
		ofs_pvt_<<uint16_t(msg.flags)<<",";
		ofs_pvt_<<uint16_t(msg.flags2)<<",";
		ofs_pvt_<<uint16_t(msg.numSV)<<",";
		ofs_pvt_<<msg.lon*1e-7<<",";
		ofs_pvt_<<msg.lat*1e-7<<",";
		ofs_pvt_<<msg.height*1e-3<<",";
		ofs_pvt_<<msg.hMSL*1e-3<<",";
		ofs_pvt_<<msg.hAcc*1e-3<<",";
		ofs_pvt_<<msg.vAcc*1e-3<<",";
		ofs_pvt_<<msg.velN*1e-3<<",";
		ofs_pvt_<<msg.velE*1e-3<<",";
		ofs_pvt_<<msg.velD*1e-3<<",";
		ofs_pvt_<<msg.gSpeed*1e-3<<",";
		ofs_pvt_<<msg.headMot*1e-5<<",";
		ofs_pvt_<<msg.sAcc*1e-3<<",";
		ofs_pvt_<<msg.headAcc*1e-5<<",";
		ofs_pvt_<<msg.pDOP*1e-2<<",";
		ofs_pvt_<<msg.headVeh*1e-5<<",";
		ofs_pvt_<<msg.magDec<<",";
		ofs_pvt_<<msg.magAcc;
		
		std::cerr<<"Wrote PVT to log file"<<std::endl;
	}

	void UBLOX_LOG::relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID)
	{
		std::cerr<<"PVT Callback, F9PID: "<<f9PID<<std::endl;
		ublox::NAV_RELPOSNED_t msg = ubx_msg.NAV_RELPOSNED;

		ofs_rp_<<std::endl;

		ofs_rp_<<msg.refStationId<<",";
		ofs_rp_<<msg.iTow<<",";
		ofs_rp_<<(msg.relPosN+msg.relPosHPN*1e-2)*1e-2<<",";
		ofs_rp_<<(msg.relPosE+msg.relPosHPE*1e-2)*1e-2<<",";
		ofs_rp_<<(msg.relPosD+msg.relPosHPD*1e-2)*1e-2<<",";
		ofs_rp_<<(msg.relPosLength+msg.relPosHPLength*1e-2)*1e-2<<",";
		ofs_rp_<<msg.relPosHeading<<",";
		ofs_rp_<<msg.accN<<",";
		ofs_rp_<<msg.accE<<",";
		ofs_rp_<<msg.accD<<",";
		ofs_rp_<<msg.accLength<<",";
		ofs_rp_<<msg.accHeading<<",";
		ofs_rp_<<msg.flags.gnssFixOk<<",";
		ofs_rp_<<msg.flags.diffSoln<<",";
		ofs_rp_<<msg.flags.relPosValid<<",";
		ofs_rp_<<msg.flags.floatCarrSoln<<",";
		ofs_rp_<<msg.flags.fixedCarrSoln<<",";
		ofs_rp_<<msg.flags.isMoving<<",";
		ofs_rp_<<msg.flags.refPosMiss<<",";
		ofs_rp_<<msg.flags.refObsMiss<<",";
		ofs_rp_<<msg.flags.relPosHeadingValid<<",";
		ofs_rp_<<msg.flags.relPosNormalized<<",";
		
	}

	void UBLOX_LOG::setlog_dir(const std::string &log_dir)
	{
		if(ofs_pvt_.is_open())
		{
			ofs_pvt_.close();
		}
		ofs_pvt_.open(log_dir+"pvt.csv", std::ofstream::trunc);
		ofs_pvt_<<"iTOW,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,flags2,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,headVeh,magDec,magAcc";
		

		if(ofs_rp_.is_open())
		{
			ofs_rp_.close();
		}
		ofs_rp_.open(log_dir+"relpos.csv", std::ofstream::trunc);
		ofs_rp_<<"refStationID,iTOW,relPosN,relPosE,relPosD,relPosLength,relPosHeading,accN,accE,accD,accLength,accHeading,gnssFixOk,diffSoln,relPosValid,floatCarrSoln,fixedCarrSoln,isMoving,refPosMiss,refObsMiss,relPosHeadingValid,relPosNormalized";
	}
	
	void UBLOX_LOG::connectCallbacks()
	{
		createCallback(CLASS_NAV, NAV_PVT, &UBLOX_LOG::pvtCB, this);
		createCallback(CLASS_NAV, NAV_RELPOSNED, &UBLOX_LOG::relposCB, this);
	}	
}

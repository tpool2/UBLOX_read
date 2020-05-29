#include <UBLOX/ublox_log.h>

namespace ublox
{
	void UBLOX_LOG::pvtCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID)
	{
		std::cerr<<"PVT Callback, F9PID: "<<uint16_t(f9PID)<<std::endl;
		
		switch (f9PID)
		{
		case 0:
			writePVT(ubx_msg, ofs_pvt_);
			pvt_ = ubx_msg.NAV_PVT;
			break;
		case 1:
			writePVT(ubx_msg, ofs_base_pvt_);
			break;
		default:
			std::cerr<<"Unknown F9PID: "<<f9PID<<"\n";
			break;
		}
	}

	void UBLOX_LOG::relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID)
	{
		std::cerr<<"Rel Pos Callback, F9PID: "<< uint16_t(f9PID)<<std::endl;
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
		ofs_rp_<<msg.flags.relPosNormalized;

		if(msg.iTow==pvt_.iTOW)
		{
			ofs_rp_<<",";
			ofs_rp_<<pvt_.year<<",";
			ofs_rp_<<uint16_t(pvt_.month)<<",";
			ofs_rp_<<uint16_t(pvt_.day)<<",";
			ofs_rp_<<uint16_t(pvt_.hour)<<",";
			ofs_rp_<<uint16_t(pvt_.min)<<",";
			ofs_rp_<<uint16_t(pvt_.sec);
			ofs_rp_<<uint16_t(pvt_.valid)<<",";
			ofs_rp_<<pvt_.tAcc<<",";
			ofs_rp_<<pvt_.nano<<",";
			ofs_rp_<<uint16_t(pvt_.fixType)<<",";
		}
		
	}

	void UBLOX_LOG::setlog_dir(const std::string &log_dir)
	{
		if(ofs_pvt_.is_open())
		{
			ofs_pvt_.close();
		}
		ofs_pvt_.open(log_dir+"pvt.csv", std::ofstream::trunc);
		ofs_pvt_<<"iTOW,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,flags2,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,headVeh,magDec,magAcc";

		if(ofs_base_pvt_.is_open())
		{
			ofs_base_pvt_.close();
		}
		ofs_base_pvt_.open(log_dir_+"base.csv", std::ofstream::trunc);
		ofs_base_pvt_<<"iTOW,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,flags2,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,headVeh,magDec,magAcc";

		if(ofs_rp_.is_open())
		{
			ofs_rp_.close();
		}
		ofs_rp_.open(log_dir+"relpos.csv", std::ofstream::trunc);
		ofs_rp_<<"refStationID,iTOW,relPosN,relPosE,relPosD,relPosLength,relPosHeading,accN,accE,accD,accLength,accHeading,gnssFixOk,diffSoln,relPosValid,floatCarrSoln,fixedCarrSoln,isMoving,refPosMiss,refObsMiss,relPosHeadingValid,relPosNormalized,year,month,day,hour,min,sec,valid,tAcc,nano,fixType";
	}
	
	void UBLOX_LOG::connectCallbacks()
	{
		createCallback(CLASS_NAV, NAV_PVT, &UBLOX_LOG::pvtCB, this);
		createCallback(CLASS_NAV, NAV_RELPOSNED, &UBLOX_LOG::relposCB, this);
	}

	void UBLOX_LOG::writePVT(const ublox::UBX_message_t &ubx_msg, std::ofstream &ofs)
	{
		ublox::NAV_PVT_t msg = ubx_msg.NAV_PVT;
		
		ofs<<std::endl;
		ofs<<msg.iTOW<<",";
		ofs<<msg.year<<",";
		ofs<<uint16_t(msg.month)<<",";
		ofs<<uint16_t(msg.day)<<",";
		ofs<<uint16_t(msg.hour)<<",";
		ofs<<uint16_t(msg.min)<<",";
		ofs<<uint16_t(msg.sec)<<",";
		ofs<<uint16_t(msg.valid)<<",";
		ofs<<msg.tAcc<<",";
		ofs<<msg.nano<<",";
		ofs<<uint16_t(msg.fixType)<<",";
		ofs<<uint16_t(msg.flags)<<",";
		ofs<<uint16_t(msg.flags2)<<",";
		ofs<<uint16_t(msg.numSV)<<",";
		ofs<<msg.lon*1e-7<<",";
		ofs<<msg.lat*1e-7<<",";
		ofs<<msg.height*1e-3<<",";
		ofs<<msg.hMSL*1e-3<<",";
		ofs<<msg.hAcc*1e-3<<",";
		ofs<<msg.vAcc*1e-3<<",";
		ofs<<msg.velN*1e-3<<",";
		ofs<<msg.velE*1e-3<<",";
		ofs<<msg.velD*1e-3<<",";
		ofs<<msg.gSpeed*1e-3<<",";
		ofs<<msg.headMot*1e-5<<",";
		ofs<<msg.sAcc*1e-3<<",";
		ofs<<msg.headAcc*1e-5<<",";
		ofs<<msg.pDOP*1e-2<<",";
		ofs<<msg.headVeh*1e-5<<",";
		ofs<<msg.magDec<<",";
		ofs<<msg.magAcc;

		std::cerr<<"Wrote PVT to log file"<<std::endl;
	}	

	void UBLOX_LOG::evalF9PID(uint8_t f9PID)
	{
		if(ofs_pvt_.is_open())
		{
			ofs_pvt_.close();
		}
		switch(f9PID)
		{
			case 0:
				ofs_pvt_.open(log_dir_+"pvt.csv", std::ofstream::app);
			case 1:
				ofs_pvt_.open(log_dir_+"base_pvt.csv", std::ofstream::app);
		}
	}
}

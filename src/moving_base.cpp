#include <iostream>
#include <fstream>
#include <string>
#include <UBLOX/ublox.h>
#include <UBLOX/ublox_log.h>

int main(int argc, char* argv[])
{
	std::string serial_port = argv[1];
	int message_rate = 10;
	ublox::UBLOX *ublox_ = new ublox::UBLOX(serial_port, message_rate);
	
	std::string *local_host = new std::string[1];
	uint16_t *local_port = new uint16_t[1];
	std::string *remote_host = new std::string[1];
	uint16_t *remote_port = new uint16_t[1];
	int rover_quantity = 1;

	local_host[0]="169.254.248.104";
	local_port[0]=16140;
	remote_host[0]="169.254.248.104";
	remote_port[0]=16145;

	ublox::GNSS_CONSTELLATION_t constellation;
	constellation.gps_enable = 1;
	constellation.glonas_enable = 1;
	constellation.beidou_enable = 1;
	constellation.galileo_enable = 1;

	int surveytime = 0;
	int surveyacc = 0;
	uint8_t dynamic_model = 0;
	std::string base_type = "moving";

	ublox::UBLOX_LOG *logger = new ublox::UBLOX_LOG(ublox_, "/home/pi/base");

	ublox_->initBase(local_host, local_port, remote_host, remote_port, base_type, rover_quantity, constellation, surveytime, surveyacc, dynamic_model);
	
	char temp;

	std::cin >> temp; 
	
	return 0;
}

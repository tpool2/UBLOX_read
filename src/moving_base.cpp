#include <iostream>
#include <fstream>
#include <string>
#include <UBLOX/ublox.h>

int main(int argc, char* argv[])
{
	std::cout<<"Hello World!"<<std::endl;
	std::string serial_port = "/dev/ttyACM0";
	int message_rate = 10;
	ublox::UBLOX *ublox_ = new ublox::UBLOX(serial_port, message_rate);
	
	std::string *local_host = new std::string[0];
	uint16_t *local_port = new uint16_t[0];
	std::string *remote_host = new std::string[0];
	uint16_t *remote_port = new uint16_t[0];
	int rover_quantity = 0;

	ublox::GNSS_CONSTELLATION_t constellation;
	constellation.gps_enable = 1;
	constellation.glonas_enable = 1;
	constellation.beidou_enable = 1;
	constellation.galileo_enable = 1;

	int surveytime = 0;
	int surveyacc = 0;
	uint8_t dynamic_model = 0;
	std::string base_type = "moving";

	ublox_->initBase(local_host, local_port, remote_host, remote_port, base_type, rover_quantity, constellation, surveytime, surveyacc, dynamic_model);

	return 0;
}

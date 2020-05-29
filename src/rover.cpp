#include <UBLOX/ublox.h>
#include <UBLOX/ublox_log.h>
#include <string>

int main(int argc, char* argv[])
{
	std::string serial_port = argv[1];
	int message_rate = 10;

	ublox::UBLOX *ublox_ = new ublox::UBLOX(serial_port, message_rate);

	std::string local_host = "10.0.0.189";
	uint16_t local_port = 16145;
	std::string remote_host = "10.0.0.189";
	uint16_t remote_port = 16140;

	ublox::GNSS_CONSTELLATION_t constellation;
	constellation.gps_enable = 1;
	constellation.glonas_enable = 1;
	constellation.beidou_enable = 1;
	constellation.galileo_enable = 1;

	uint8_t dynamic_model = 0;

	ublox::UBLOX_LOG *logger = new ublox::UBLOX_LOG(ublox_, "/Users/taylorpool/log_files/macrover");

	ublox_->initRover(local_host, local_port, remote_host, remote_port, constellation, dynamic_model);

	char temp;

	std::cin >> temp;

	delete ublox_;
	delete logger;

	return 0;
}

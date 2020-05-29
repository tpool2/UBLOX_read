#ifndef NMEA_H_
#define NMEA_H_

#include <iostream>
#include "UBLOX/parsers/nmea_defs.h"
#include "async_comm/serial.h"

namespace ublox
{
	class NMEA
	{
		public:
			NMEA(async_comm::Serial &serial) : serial_(serial){};
			~NMEA();
		private:

			/**
			 * @brief returns true if the byte was successfully processed
			 */
			bool read_cb(uint8_t byte, uint8_t f9PID);

			/**
			 * @brief returns true if message is successfully decoded
			 */
			bool decode_message(uint8_t f9PID);

			/**
			 * @brief Returns true if nmea message is being parsed
			 */
			bool parsing_message();

			NMEA_PARSE_STATE_t parse_state_;
			async_comm::Serial &serial_;
	};
}


#endif

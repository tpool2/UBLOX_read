#ifndef NMEA_H_
#define NMEA_H_

#include <iostream>

namespace ublox
{
	class NMEA
	{
		public:
			NMEA();
			~NMEA();
		private:
			bool read_cb(uint8_t byte, uint8_t f9PID);
			bool decode_message(uint8_t f9PID);
	}
}


#endif

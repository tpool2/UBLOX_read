#include <UBLOX/ublox.h>
#include <iostream>
#include <fstream>
#include <string>

namespace ublox
{
	class UBLOX_LOG
	{
		public:
			UBLOX_LOG(ublox::UBLOX *ublox, const std::string &log_dir)
			{
				ublox_ = ublox;
				setlog_dir(log_dir);
				connectCallbacks();
			};
			
			~UBLOX_LOG();

		private:
			ublox::UBLOX* ublox_ = nullptr;
			std::string log_dir_;

			std::ofstream ofs_pvt_;
			std::ofstream ofs_rp_;

			void pvtCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID=0);
			
			void relposCB(const ublox::UBX_message_t &ubx_msg, uint8_t f9PID=0);
			
			void setlog_dir(const std::string &log_dir);
			
			void connectCallbacks();

			template<class M> void createCallback(uint8_t cls, uint8_t type, void(ublox::UBLOX_LOG::*fp)(const M &msg, uint8_t), ublox::UBLOX_LOG *obj, uint8_t f9PID=0)
			{
				do
				{
					auto trampoline = [obj, fp](uint8_t _class, uint8_t _type, const ublox::UBX_message_t &ubx_msg, uint8_t f9PID=0)
					{
						(obj->*fp)(ubx_msg, f9PID);
					};

					this->ublox_->registerUBXCallback(cls, type, trampoline);
				} while(0);
			};
	};	
}

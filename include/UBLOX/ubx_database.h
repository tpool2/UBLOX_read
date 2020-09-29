#ifndef UBX_DATABASE_H
#define UBX_DATABASE_H
#include <cstdint>
#include <map>
#include "UBLOX/ubx_defs.h"

namespace ublox::ubx
{
    class DatabaseInterface
    {   
        protected:
            class DatabaseNodeInterface
            {

            };

        public:
            virtual bool has(uint8_t message_class, uint8_t message_id);

    };

    class Database: public DatabaseInterface
    {
        private:
            class DatabaseNode: private DatabaseNodeInterface
            {
                private:
                    int length;
                
                public:
                    DatabaseNode(int len=1)
                    {
                        length = len;
                    }

                    uint16_t get_length() const;
            };

            std::map<uint8_t, DatabaseNode> ACK_ID_Map
            {
                {kACK_ACK, DatabaseNode(2)},
                {kACK_NACK, DatabaseNode(2)},
            };

            std::map<uint8_t, DatabaseNode> MGA_ID_Map
            {
                {kMGA_GPS, DatabaseNode(68)},
            };

            std::map<uint8_t, std::map<uint8_t, DatabaseNode>> UBX_CLASS_Map
            {
                {kCLASS_ACK, ACK_ID_Map},
                {kCLASS_MGA, MGA_ID_Map},
            };

        public:
            Database()
            {

            };
            bool has(uint8_t message_class, uint8_t message_id) override;
    };
}
#endif
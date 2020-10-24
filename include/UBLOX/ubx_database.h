#ifndef UBX_DATABASE_H
#define UBX_DATABASE_H
#include <cstdint>
#include <map>
#include <memory>
#include "UBLOX/ubx_defs.h"

namespace ublox::ubx
{
    class DatabaseNodeInterface
    {
        public:
            virtual bool length_matches(int message_length) const = 0;
    };

    class DatabaseInterface
    {   
        public:
            virtual bool has(uint8_t message_class, uint8_t message_id) = 0;
            virtual std::shared_ptr<DatabaseNodeInterface> get_node(uint8_t message_class, uint8_t message_id) = 0;
    };

    class Database: public DatabaseInterface
    {
        private:
            std::shared_ptr<DatabaseNodeInterface> null_node;
            
            class DatabaseNode: public DatabaseNodeInterface
            {
                private:
                    int length;
                    int linear_length;
                
                public:
                    DatabaseNode(int length=1, int linear_length=0)
                    {
                        this->length = length;
                        this->linear_length = linear_length;
                    }
                    bool length_matches(int payload_length) const;
            };
            typedef std::shared_ptr<DatabaseNode> DatabaseNodePtr;

            std::map<uint8_t, DatabaseNodePtr> NAV_ID_Map
            {
                {kNAV_ORB, std::make_shared<DatabaseNode>(8,6)},
            };

            std::map<uint8_t, DatabaseNodePtr> ACK_ID_Map
            {
                {kACK_ACK, std::make_shared<DatabaseNode>(2)},
                {kACK_NACK, std::make_shared<DatabaseNode>(2)},
            };

            std::map<uint8_t, DatabaseNodePtr> MGA_ID_Map
            {
                {kMGA_GPS, std::make_shared<DatabaseNode>(68)},
            };

            std::map<uint8_t, std::map<uint8_t, DatabaseNodePtr>> UBX_CLASS_Map
            {
                {kCLASS_ACK, ACK_ID_Map},
                {kCLASS_MGA, MGA_ID_Map},
                {kCLASS_NAV, NAV_ID_Map},
            };

        public:
            Database()
            {
                null_node = std::make_shared<DatabaseNode>(0,0);
            };
            bool has(uint8_t message_class, uint8_t message_id) override;
            virtual std::shared_ptr<DatabaseNodeInterface> get_node(uint8_t message_class, uint8_t message_id);
    };
}
#endif
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
            virtual bool has(uint8_t message_class) = 0;
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
            typedef std::map<uint8_t, DatabaseNodePtr> DatabaseMap;
            
            DatabaseMap ACK_ID_Map
            {
                {kACK_ACK, std::make_shared<DatabaseNode>(2)},
                {kACK_NACK, std::make_shared<DatabaseNode>(2)},
            };

            DatabaseMap CFG_ID_Map
            {
                {kCFG_VALDEL, std::make_shared<DatabaseNode>(4,4)},
                {kCFG_VALGET, std::make_shared<DatabaseNode>(4,4)},
                {kCFG_VALSET, std::make_shared<DatabaseNode>(4,1)},
            };

            DatabaseMap INF_ID_Map
            {

            };

            DatabaseMap LOG_ID_Map
            {

            };

            DatabaseMap MGA_ID_Map
            {
                {kMGA_GPS, std::make_shared<DatabaseNode>(68)},
            };

            DatabaseMap MON_ID_Map
            {

            };

            DatabaseMap NAV_ID_Map
            {
                {kNAV_ORB, std::make_shared<DatabaseNode>(8,6)},
            };

            DatabaseMap RXM_ID_Map
            {

            };

            DatabaseMap SEC_ID_Map
            {

            };

            DatabaseMap TIM_ID_Map
            {

            };

            DatabaseMap UPD_ID_Map
            {

            };

            std::map<uint8_t, DatabaseMap> UBX_CLASS_Map
            {
                {kCLASS_ACK, ACK_ID_Map},
                {kCLASS_CFG, CFG_ID_Map},
                {kCLASS_INF, INF_ID_Map},
                {kCLASS_LOG, LOG_ID_Map},
                {kCLASS_MGA, MGA_ID_Map},
                {kCLASS_MON, MON_ID_Map},
                {kCLASS_NAV, NAV_ID_Map},
                {kCLASS_RXM, RXM_ID_Map},
                {kCLASS_SEC, SEC_ID_Map},
                {kCLASS_TIM, TIM_ID_Map},
                {kCLASS_UPD, UPD_ID_Map},
            };

        public:
            Database()
            {
                null_node = std::make_shared<DatabaseNode>(0,0);
            };
            bool has(uint8_t message_class, uint8_t message_id) override;
            bool has(uint8_t message_class) override;
            virtual std::shared_ptr<DatabaseNodeInterface> get_node(uint8_t message_class, uint8_t message_id);
    };
}
#endif
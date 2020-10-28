#ifndef UBX_DEFS_H
#define UBX_DEFS_H
#include <cstdint>
#include <cstddef>
#include <string>
#include <tuple>
#include <vector>

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

namespace ublox::ubx
{
static constexpr size_t BUFFER_SIZE = 1024;
static constexpr size_t UBX_MESSAGE_BUFFER_SIZE = BUFFER_SIZE+8;
enum {
    FIX_TYPE_NO_FIX = 0x00,
    FIX_TYPE_DEAD_RECKONING = 0x01,
    FIX_TYPE_2D = 0x02,
    FIX_TYPE_3D = 0x03,
    FIX_TYPE_GPS_AND_DEAD_RECKONING = 0x04,
    FIX_TYPE_TIME_ONLY = 0x05,
};

enum {
    kSTART_BYTE_1 = 0xB5,
    kSTART_BYTE_2 = 0x62,
};

enum {
    kCLASS_NAV = 0x01, //    Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    kCLASS_RXM = 0x02, //    Receiver Manager Messages: Satellite Status, RTC Status
    kCLASS_INF = 0x04, //    Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    kCLASS_ACK = 0x05, //    Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
    kCLASS_CFG = 0x06, //    Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
    kCLASS_UPD = 0x09, //    Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    kCLASS_MON = 0x0A, //    Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    kCLASS_AID = 0x0B, //    AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    kCLASS_TIM = 0x0D, //    Timing Messages: Time Pulse Output, Time Mark Results
    kCLASS_ESF = 0x10, //    External Sensor Fusion Messages: External Sensor Measurements and Status Information
    kCLASS_MGA = 0x13, //    Multiple GNSS Assistance Messages: Assistance data for various GNSS
    kCLASS_LOG = 0x21, //    Logging Messages: Log creation, deletion, info and retrieva
    kCLASS_SEC = 0x27, //    Used for security features of the receiver.
    kCLASS_RTCM = 0XF5, //   RTCM messages (definitely for output possibly for all of the input messages)
};

enum {
    kACK_ACK = 0x01, // Message Acknowledged
    kACK_NACK = 0x00, // Message Not-Acknowledged
};

enum {
    kAID_ALM = 0x30, // Poll GPS Aiding Almanac Data
    kAID_AOP = 0x33, // AssistNow Autonomous data
    kAID_EPH = 0x31, // GPS Aiding Ephemeris Input/Output Message
    kAID_HUI = 0x02, // Poll GPS Health, UTC, ionosphere parameters
    kAID_INI = 0x01, // Aiding position, time, frequency, clock drift
};

enum {
    kCFG_ANT = 0x13, 	// Get/Set Antenna Control Settings
    kCFG_BATCH = 0x93, 	// Get/Set Get/Set data batching configuration
    kCFG_CFG = 0x09, 	// Command Clear, Save and Load configurations
    kCFG_DAT = 0x06, 	// Get The currently defined Datum
    kCFG_DGNSS = 0x70, 	// Get/Set DGNSS configuration
    kCFG_DOSC = 0x61, 	// Get/Set Disciplined oscillator configuration
    kCFG_DYNSEED = 0x85, // Set Programming the dynamic seed for the host...
    kCFG_ESRC = 0x60, 	// Get/Set External synchronization source configuration
    kCFG_FIXSEED = 0x84, // Set Programming the fixed seed for host...
    kCFG_GEOFENCE = 0x69,// Get/Set Geofencing configuration
    kCFG_GNSS = 0x3E, 	// Get/Set GNSS system configuration
    kCFG_HNR = 0x5C, 	// Get/Set High Navigation Rate Settings
    kCFG_INF = 0x02, 	// Poll Request Poll configuration for one protocol
    kCFG_ITFM = 0x39, 	// Get/Set Jamming/Interference Monitor configuration
    kFG_LOGFILTER = 0x47,// Get/Set Data Logger Configuration
    kCFG_MSG = 0x01, 	// Poll Request Poll a message configuration
    kCFG_NAV5 = 0x24, 	// Get/Set Navigation Engine Settings
    kCFG_NAVX5 = 0x23, 	// Get/Set Navigation Engine Expert Settings
    kCFG_NMEA = 0x17, 	// Get/Set NMEA protocol configuration (deprecated)
    kCFG_ODO = 0x1E, 	// Get/Set Odometer, Low_speed COG Engine Settings
    kCFG_PM2 = 0x3B, 	// Get/Set Extended Power Management configuration
    kCFG_PMS = 0x86, 	// Get/Set Power Mode Setup
    kCFG_PRT = 0x00, 	// Poll Request Polls the configuration for one I/O Port
    kCFG_PWR = 0x57, 	// Set Put receiver in a defined power state.
    kCFG_RATE = 0x08, 	// Get/Set Navigation/Measurement Rate Settings
    kCFG_RINV = 0x34, 	// Get/Set Contents of Remote Inventory
    kCFG_RST = 0x04, 	// Command Reset Receiver / Clear Backup Data Structures
    kCFG_RXM = 0x11, 	// Get/Set RXM configuration
    kCFG_SBAS = 0x16, 	// Get/Set SBAS Configuration
    kCFG_SMGR = 0x62, 	// Get/Set Synchronization manager configuration
    kCFG_TMODE2 = 0x3D, 	// Get/Set Time Mode Settings 2
    kCFG_TMODE3 = 0x71, 	// Get/Set Time Mode Settings 3
    kCFG_TP5 = 0x31, 	// Poll Request Poll Time Pulse Parameters for Time Pulse 0
    kCFG_TXSLOT = 0x53, 	// Set TX buffer time slots configuration
    kCFG_USB = 0x1B, 	// Get/Set USB Configuration
    kCFG_VALDEL = 0x8C, //Deletes values corresponding to...
    kCFG_VALGET = 0x8B, //Get configuration items
    kCFG_VALSET = 0x8A, //Set values correpsonding to provided...
};
enum {
    kNAV_AOPSTATUS = 0x60,	// Periodic/Polled AssistNow Autonomous Status
    kNAV_ATT = 0x05,		// Periodic/Polled Attitude Solution
    kNAV_CLOCK = 0x22,		// Periodic/Polled Clock Solution
    kNAV_DGPS = 0x31,		// Periodic/Polled DGPS Data Used for NAV
    kNAV_DOP = 0x04,		// Periodic/Polled Dilution of precision
    kNAV_EOE = 0x61,		// Periodic End Of Epoch
    kNAV_GEOFENCE = 0x39,	// Periodic/Polled Geofencing status
    kNAV_HPPOSECEF = 0x13,	// Periodic/Polled High Precision Position Solution in ECEF
    kNAV_HPPOSLLH = 0x14,	// Periodic/Polled High Precision Geodetic Position Solution
    kNAV_ODO = 0x09,		// Periodic/Polled Odometer Solution
    kNAV_ORB = 0x34,		// Periodic/Polled GNSS Orbit Database Info
    kNAV_POSECEF = 0x01,		// Periodic/Polled Position Solution in ECEF
    kNAV_POSLLH = 0x02,		// Periodic/Polled Geodetic Position Solution
    kNAV_PVT = 0x07,		// Periodic/Polled Navigation Position Velocity Time Solution
    kNAV_RELPOSNED = 0x3C,	// Periodic/Polled Relative Positioning Information in NED frame
    kNAV_RESETODO = 0x10,	// Command Reset odometer
    kNAV_SAT = 0x35,		// Periodic/Polled Satellite Information
    kNAV_SBAS = 0x32,		// Periodic/Polled SBAS Status Data
    kNAV_SOL = 0x06,		// Periodic/Polled Navigation Solution Information
    kNAV_STATUS = 0x03,		// Periodic/Polled Receiver Navigation Status
    kNAV_SVINFO = 0x30,		// Periodic/Polled Space Vehicle Information
    kNAV_SVIN = 0x3B,		// Periodic/Polled Survey-in data
    kNAV_TIMEBDS = 0x24,		// Periodic/Polled BDS Time Solution
    kNAV_TIMEGAL = 0x25,		// Periodic/Polled Galileo Time Solution
    kNAV_TIMEGLO = 0x23,		// Periodic/Polled GLO Time Solution
    kNAV_TIMEGPS = 0x20,		// Periodic/Polled GPS Time Solution
    kNAV_TIMELS = 0x26,		// Periodic/Polled Leap second event information
    kNAV_TIMEUTC = 0x21,		// Periodic/Polled UTC Time Solution
    kNAV_VELECEF = 0x11,		// Periodic/Polled Velocity Solution in ECEF
    kNAV_VELNED = 0x12,		// Periodic/Polled Velocity Solution in NED
    kNAV_SIG = 0x43,         // Periodic/Polled signal information
};

enum
{
    kMGA_ACK = 0x60,
    kMGA_BDS = 0x03,
    kMGA_DBD = 0x80,
    kMGA_GAL = 0x02,
    kMGA_GLO = 0x06,
    kMGA_GPS = 0x00,
    kMGA_INI = 0x40,
    kMGA_QZSS = 0x05,
};

enum {
    kMON_COMMS = 0x36,
    kMON_GNSS = 0x28,
    kMON_HW2 = 0x0B,
    kMON_HW3 = 0x37,
    kMON_HW = 0x09,
    kMON_IO = 0x02,
    kMON_MSGPP = 0x06,
    kMON_PATCH = 0x27,
    kMON_RF = 0x38,
    kMON_RXBUF = 0x07,
    kMON_RXR = 0x21,
    kMON_TXBUF = 0x08,
    kMON_VER = 0x04
};

enum {
    kRXM_RAWX = 0x15,         // Multi-GNSS Raw Measurement Data
    kRXM_SFRBX = 0x13,         // Broadcast Navigation Data Subframe (ephemeris)
    kRXM_RTCM = 0x32,        // RTCM Data acknowledgment after proper parsing
    kRXM_MEASX = 0x14        // Satellite Measurements for RRLP (Radio Resource Location Services Protocol)
};

typedef enum {
    kSTART,
    kGOT_START_FRAME,
    kGOT_CLASS,
    kGOT_MSG_ID,
    kGOT_LENGTH1,
    kGOT_LENGTH2,
    kGOT_PAYLOAD,
    kGOT_CK_A,
    kGOT_CK_B,
    kGOT_CK_C,
    kDONE,
} parse_state_t;

typedef struct {
    uint8_t clsID;
    uint8_t msgID;
}__attribute__((packed)) ACK_ACK_t;

typedef struct {
    uint8_t clsID;
    uint8_t msgID;
}__attribute__((packed)) ACK_NACK_t;

typedef struct {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
}__attribute__((packed)) CFG_MSG_t;

typedef struct {
    enum {
        kDYNMODE_PORTABLE = 0,
        kDYNMODE_STATIONARY = 2,
        kDYNMODE_PEDESTRIAN = 3,
        kDYNMODE_AUTOMOTIVE = 4,
        kDYNMODE_SEA = 5,
        kDYNMODE_AIRBORNE_1G = 6,
        kDYNMODE_AIRBORNE_2G = 7,
        kDYNMODE_AIRBORNE_4G = 8
    };
    enum {
        kFIXMODE_2D_ONLY = 1,
        kFIXMODE_3D_ONLY = 2,
        kFIXMODE_AUTO = 3,
    };

    enum{
        kUTC_STANDARD_AUTO = 0, // receiver selects based on GNSS configuration (see GNSS time bases).
        kUTC_STANDARD_USA = 3, // UTC as operated by the U.S. Naval Observatory (USNO); derived from GPS time
        kUTC_STANDARD_RUS = 6, // UTC as operated by the former Soviet Union; derived from GLONASS time
        kUTC_STANDARD_CHN = 7, // UTC as operated by the National Time Service Center, China; derived from BeiDou time
    };

    enum {
        kMASK_DYN            = 0b00000000001, // Apply dynamic model settings
        kMASK_MINEL 	        = 0b00000000010, // Apply minimum elevation settings
        kMASK_POSFIXMODE     = 0b00000000100, // Apply fix mode settings
        kMASK_DRLIM 	        = 0b00000001000, // Reserved
        kMASK_POSMASK 	    = 0b00000010000, // Apply position mask settings
        kMASK_TIMEMASK 	    = 0b00000100000, // Apply time mask settings
        kMASK_STATICHOLDMASK = 0b00001000000, // Apply static hold settings
        kMASK_DGPSMASK 	    = 0b00010000000, // Apply DGPS settings.
        kMASK_CNOTHRESHOLD   = 0b00100000000, // Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs).
        kMASK_UTC 	        = 0b10000000000, // Apply UTC settings
    };

    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt; // (1e-2 m) Fixed altitude (mean sea level) for 2D fix mode.
    uint32_t fixedAltVar; // (0.0001 m^2)Fixed altitude variance for 2D mode.
    int8_t minElev; // (deg) Minimum Elevation for a GNSS satellite to be used in NAV
    uint8_t drLimit; // s Reserved
    uint16_t pDop; // (0.1) - Position DOP Mask to use
    uint16_t tDop; // (0.1) - Time DOP Mask to use
    uint16_t pAcc; // m Position Accuracy Mask
    uint16_t tAcc; // m Time Accuracy Mask
    uint8_t staticHoldThr; // esh cm/s Static hold threshold
    uint8_t dgnssTimeout; // s DGNSS timeout
    uint8_t cnoThreshNumS; // Vs - Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
    uint8_t cnoThresh; // dBHz C/N0 threshold for deciding whether to attempt a fix
    uint8_t reserved1[2]; //  - Reserved
    uint16_t staticHoldMax; // Dist m Static hold distance threshold (before quitting static hold)
    uint8_t utcStandard; // - UTC standard to be used:
    uint8_t reserved2[5];

}__attribute__((packed)) CFG_NAV5_t;

typedef struct {
    enum {
        kPORT_I2C = 0,
        kPORT_UART1 = 1,
        kPORT_USB = 3,
        kPORT_SPI = 4
    };
    enum {
        kCHARLEN_8BIT = 0b11000000,
        kPARITY_NONE  = 0b100000000000,
        kSTOP_BITS_1  = 0x0000
    };
    enum {
        kIN_UBX   = 0b00000001,
        kIN_NMEA  = 0b00000010,
        kIN_RTCM  = 0b00000100,
        kIN_RTCM3 = 0b00100000,
    };
    enum {
        kOUT_UBX   = 0b00000001,
        kOUT_NMEA  = 0b00000010,
        kOUT_RTCM3 = 0b00100000,
    };

    uint8_t portID;
    uint8_t reserved1;
    uint16_t txReady;
    uint32_t mode; // What the mode the serial data is on (See charlen, parity and stop bits)
    uint32_t baudrate;
    uint16_t inProtoMask; // Which input protocols are enabled
    uint16_t outProtoMask; // Which output protocols are enabled
    uint16_t flags;
    uint8_t reserved2[2];
}__attribute__((packed)) CFG_PRT_t;

typedef union {
    enum {
        kSIZE_ONE_BIT = 0x01,
        kSIZE_ONE_BYTE = 0x02,
        kSIZE_TWO_BYTE = 0x03,
        kSIZE_FOUR_BYTE = 0x04,
        kSIZE_EIGHT_BYTE = 0x05
    };

    uint8_t buffer[4];
    uint32_t keyID;
    struct {
        uint8_t msgID       : 8;
        uint8_t reserved0   : 8;
        uint8_t groupID     : 8;
        uint8_t reserved1   : 4;
        uint8_t size        : 3;
        uint8_t reserved2   : 1;
    };
}__attribute__((packed)) CFG_KEY_ID_t;

typedef union {
    uint8_t buffer[4];
    uint64_t data;
}__attribute__((packed)) CFG_DATA_t;

typedef union {
    enum {
        kRAM = 0,
        kBBR = 1,
        kFLASH = 2,
        kDEFAULT = 7,
    };
    enum {
        kREQUEST = 0,
        kPOLL  = 1,
    };

    enum {
        kVALSET_float = 2,
        kVALSET_fixed  = 3,
    };

    enum {
        kMSGOUT_RELPOSNED = 0x20910090, //Output rate of the UBX-NAV-RELPOSNED message on port USB
        kMSGOUT_PVT = 0x20910009, //Output rate of the UBX-NAV-PVT message on port USB
        kMSGOUT_POSECEF = 0x20910027, //Output rate of the UBX-NAV-POSECEF message on port USB
        kMSGOUT_VELECEF = 0x20910040, //Output rate of the UBX-NAV-VELECEF message on port USB
        kMSGOUT_RAWX = 0x209102a7, //Output rate of the UBX-RXM-RAWX message on port USB
        kMSGOUT_SFRBX = 0x20910234, //Output rate of the UBX-RXM-SFRBX message on port USB
    };

    enum { //outgoing message rates for RTCM 3x on usb type U1
        //suggested messages for stationary base
        kRTCM_1005USB = 0x209102c0, //CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
        kRTCM_1074USB = 0x20910361, //CFG-MSGOUT-RTCM_3X_TYPE1074_USB -- GPS MSM 4
        kRTCM_1084USB = 0x20910366, //CFG-MSGOUT-RTCM_3X_TYPE1084_USB -- GLONASS MSM 4
        kRTCM_1094USB = 0x2091036b, //CFG-MSGOUT-RTCM_3X_TYPE1094_USB -- Galileo MSM 4
        kRTCM_1124USB = 0x20910370, //CFG-MSGOUT-RTCM_3X_TYPE1124_USB -- Beidou MSM 4
        kRTCM_1230USB = 0x20910306, //CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2 Code-Phase Biases
        //suggested messages for mobile base
        kRTCM_4072_0USB = 0x20910301, //CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB -- UBLOX Proprietary RTCM message
        kRTCM_4072_1USB = 0x20910384, //CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB --
        kRTCM_1077USB   = 0x209102cf, //CFG-MSGOUT-RTCM_3X_TYPE1077_USB __ GPS MSM 7 (high precision)
        kRTCM_1087USB   = 0x209102d4, //CFG-MSGOUT-RTCM_3X_TYPE1087_USB __ GLONASS MSM 7 (high precision)
        kRTCM_1097USB   = 0x2091031b, //CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
        kRTCM_1127USB   = 0x209102d9, //CFG-MSGOUT-RTCM_3X_TYPE1127_USB __ Beidou MSM 7 (high precision)
        //!!!!also use RTCM_1230USB above!!!///
    };

    enum {
        kUSB_INPROT_UBX = 0x10770001, //Flag to indicate if UBX should be an input protocol on USB
        kUSB_INPROT_NMEA = 0x10770002, //Flag to indicate if NMEA should be an input protocol on USB
        kUSB_INPROT_RTCM3X = 0x10770004, //Flag to indicate if RTCM3X should be an input protocol on USB
        kUSB_OUTPROT_UBX = 0x10780001, //Flag to indicate if UBX should bean output protocol on USB
        kUSB_OUTPROT_NMEA = 0x10780002, //Flag to indicate if NMEA should bean output protocol on USB
        kUSB_OUTPROT_RTCM3X = 0x10780004, //Flag to indicate if RTCM3X should bean output protocol on USB
    };

    enum {

        kDYNMODEL = 0x20110021, //Dynamic platform model
        kDYNMODE_PORTABLE = 0,
        kDYNMODE_STATIONARY = 2,
        kDYNMODE_PEDESTRIAN = 3,
        kDYNMODE_AUTOMOTIVE = 4,
        kDYNMODE_SEA = 5,
        kDYNMODE_AIRBORNE_1G = 6,
        kDYNMODE_AIRBORNE_2G = 7,
        kDYNMODE_AIRBORNE_4G = 8,
        kDYNMODE_WRIST_WORN = 9,
        kDYNMODE_BIKE =10,
    };

    enum {
        kMSGOUT_SVIN = 0x2091008b,
        kTMODE_MODE = 0x20030001,
        kTMODE_SVIN_MIN_DUR = 0x40030010, //survey in minimum duration s
        kTMODE_SVIN_ACC_LIMIT = 0x40030011, //Survey-in position accuracy limit mm
    };

        //enum not finished, but not needed.  The rest is not needed.
    enum {
        kSIGNAL_GPS = 0x1031001f, //GPS enable
        kSIGNAL_GPS_L1 = 0x10310001, //GPS L1C/A
        kSIGNAL_GPS_L2 = 0x10310003, //GPS L2C (only on u-blox F9 platform products)
        kSIGNAL_GAL = 0x10310021, //Galileo enable
        kSIGNAL_GAL_E1 = 0x10310007, //Galileo E1
        kSIGNAL_GAL_E5B = 0x1031000a, //Galileo E5b (only on u-blox F9 platform products)
        kSIGNAL_BDS = 0x10310022, //BeiDou Enable
        kSIGNAL_BDS_B1 = 0x1031000d, //BeiDou B1I
        kSIGNAL_BDS_B2 = 0x1031000e, //BeiDou B2
    };

    enum {
        //used for nav rate messages
        kRATE_MEAS = 0x30210001, //Nominal time between GNSS measurements (e.g. 100ms results in 10Hz measurement rate, 1000ms = 1Hz measurement rate)
        kRATE_NAV = 0x30210002, //Ratio of number of measurements to number of navigation solutions
        kRATE_TIMEREF = 0x20210003, //Time system to which measurements are aligned
    };

    enum {                      // Constants for cfg-rate-timeref/ RATE_TIMEREF
        kTIME_REF_UTC = 0,
        kTIME_REF_GPS = 1,
        kTIME_REF_GLONASS = 2,
        kTIME_REF_BUIDOU = 3,
        kTIME_REF_GALILEO = 4
    };
    public:
        typedef union {
            uint8_t buffer[2];
            uint16_t position;
        }__attribute__((packed)) position_t;

        typedef struct {
            uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
            uint8_t layer;
            uint16_t position;
            CFG_KEY_ID_t cfgDataKey;
        }__attribute__((packed)) request_t;

        typedef struct {
            uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
            uint8_t layer;
            position_t position;
            CFG_KEY_ID_t cfgDataKey;
            std::string keyName;
            CFG_DATA_t cfgData;
        } response_t;

        request_t request;
        response_t response;

} CFG_VALGET_t;

typedef union {
    struct {
    uint8_t invalid_map_val   : 1;
    uint8_t got_nack            : 1;
    uint8_t got_ack             : 1;
    uint8_t got_cfg_val       : 1;
    };
    uint8_t flags;
}__attribute__((packed)) CFG_VAL_DBG_t;

typedef std::tuple<CFG_VAL_DBG_t, std::vector<CFG_VALGET_t::response_t> > CFG_VALGET_TUPLE_t;

typedef struct {
    enum {
        kRAM = 0b00000001,
        kBBR = 0b00000010,
        kFLASH = 0b00000100,
        kDEFAULT = 0b01000000,
    };

    enum {
        kVERSION_0 = 0,
        kVERSION_1 = 1,
    };

    enum {
        kVALSET_float = 2,
        kVALSET_fixed  = 3,
    };

    enum {
        kMSGOUT_RELPOSNED = 0x20910090, //Output rate of the UBX-NAV-RELPOSNED message on port USB
        kMSGOUT_PVT = 0x20910009, //Output rate of the UBX-NAV-PVT message on port USB
        kMSGOUT_POSECEF = 0x20910027, //Output rate of the UBX-NAV-POSECEF message on port USB
        kMSGOUT_VELECEF = 0x20910040, //Output rate of the UBX-NAV-VELECEF message on port USB
        kMSGOUT_RAWX = 0x209102a7, //Output rate of the UBX-RXM-RAWX message on port USB
        kMSGOUT_SFRBX = 0x20910234, //Output rate of the UBX-RXM-SFRBX message on port USB
        kMSGOUT_RXM_RTCM = 0x2091026b
    };

    enum { //outgoing message rates for RTCM 3x on usb type U1
        //suggested messages for stationary base
        kRTCM_1005USB = 0x209102c0, //CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
        kRTCM_1074USB = 0x20910361, //CFG-MSGOUT-RTCM_3X_TYPE1074_USB -- GPS MSM 4
        kRTCM_1084USB = 0x20910366, //CFG-MSGOUT-RTCM_3X_TYPE1084_USB -- GLONASS MSM 4
        kRTCM_1094USB = 0x2091036b, //CFG-MSGOUT-RTCM_3X_TYPE1094_USB -- Galileo MSM 4
        kRTCM_1124USB = 0x20910370, //CFG-MSGOUT-RTCM_3X_TYPE1124_USB -- Beidou MSM 4
        kRTCM_1230USB = 0x20910306, //CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2 Code-Phase Biases
        //suggested messages for mobile base
        kRTCM_4072_0USB = 0x20910301, //CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB -- UBLOX Proprietary RTCM message
        kRTCM_4072_1USB = 0x20910384, //CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB --
        kRTCM_1077USB   = 0x209102cf, //CFG-MSGOUT-RTCM_3X_TYPE1077_USB __ GPS MSM 7 (high precision)
        kRTCM_1087USB   = 0x209102d4, //CFG-MSGOUT-RTCM_3X_TYPE1087_USB __ GLONASS MSM 7 (high precision)
        kRTCM_1097USB   = 0x2091031b, //CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
        kRTCM_1127USB   = 0x209102d9, //CFG-MSGOUT-RTCM_3X_TYPE1127_USB __ Beidou MSM 7 (high precision)
        //!!!!also use RTCM_1230USB above!!!///
    };

    enum {
        kUSB_INPROT_UBX = 0x10770001, //Flag to indicate if UBX should be an input protocol on USB
        kUSB_INPROT_NMEA = 0x10770002, //Flag to indicate if NMEA should be an input protocol on USB
        kUSB_INPROT_RTCM3X = 0x10770004, //Flag to indicate if RTCM3X should be an input protocol on USB
        kUSB_OUTPROT_UBX = 0x10780001, //Flag to indicate if UBX should bean output protocol on USB
        kUSB_OUTPROT_NMEA = 0x10780002, //Flag to indicate if NMEA should bean output protocol on USB
        kUSB_OUTPROT_RTCM3X = 0x10780004, //Flag to indicate if RTCM3X should bean output protocol on USB
    };

    enum {

        kDYNMODEL = 0x20110021, //Dynamic platform model
        kDYNMODE_PORTABLE = 0,
        kDYNMODE_STATIONARY = 2,
        kDYNMODE_PEDESTRIAN = 3,
        kDYNMODE_AUTOMOTIVE = 4,
        kDYNMODE_SEA = 5,
        kDYNMODE_AIRBORNE_1G = 6,
        kDYNMODE_AIRBORNE_2G = 7,
        kDYNMODE_AIRBORNE_4G = 8,
        kDYNMODE_WRIST_WORN = 9,
        kDYNMODE_BIKE =10,
    };

    enum {
        kMSGOUT_SVIN = 0x2091008b,
        kTMODE_MODE = 0x20030001,
        kTMODE_SVIN_MIN_DUR = 0x40030010, //survey in minimum duration s
        kTMODE_SVIN_ACC_LIMIT = 0x40030011, //Survey-in position accuracy limit mm
    };

        //enum not finished, but not needed.  The rest is not needed.
    enum {
        kSIGNAL_GPS = 0x1031001f, //GPS enable
        kSIGNAL_GPS_L1 = 0x10310001, //GPS L1C/A
        kSIGNAL_GPS_L2 = 0x10310003, //GPS L2C (only on u-blox F9 platform products)
        kSIGNAL_GAL = 0x10310021, //Galileo enable
        kSIGNAL_GAL_E1 = 0x10310007, //Galileo E1
        kSIGNAL_GAL_E5B = 0x1031000a, //Galileo E5b (only on u-blox F9 platform products)
        kSIGNAL_BDS = 0x10310022, //BeiDou Enable
        kSIGNAL_BDS_B1 = 0x1031000d, //BeiDou B1I
        kSIGNAL_BDS_B2 = 0x1031000e, //BeiDou B2I
        kSIGNAL_GLO = 0x10310025,
        kSIGNAL_GLO_L1 = 0x10310018,
        kSIGNAL_GLO_L2 = 0x1031001a,
        kSIGNAL_QZSS = 0x10310024,
        kSIGNAL_QZSS_L1CA =  0x10310012,
        kSIGNAL_QZSS_L2C =  0x10310015
    };

    enum {
        //used for nav rate messages
        kRATE_MEAS = 0x30210001, //Nominal time between GNSS measurements (e.g. 100ms results in 10Hz measurement rate, 1000ms = 1Hz measurement rate)
        kRATE_NAV = 0x30210002, //Ratio of number of measurements to number of navigation solutions
        kRATE_TIMEREF = 0x20210003, //Time system to which measurements are aligned
    };

    enum {                      // Constants for cfg-rate-timeref/ RATE_TIMEREF
        kTIME_REF_UTC = 0,
        kTIME_REF_GPS = 1,
        kTIME_REF_GLONASS = 2,
        kTIME_REF_BUIDOU = 3,
        kTIME_REF_GALILEO = 4
    };

    uint8_t version;
    uint8_t layer;
    uint8_t reserved1[2];
    CFG_KEY_ID_t cfgDataKey;

    union
    {
        uint8_t bytes[8];
        uint16_t half_word[4];
        uint32_t word[2];
        uint64_t two_word;
    } cfgData;


}__attribute__((packed)) CFG_VALSET_t;

typedef struct
{
    enum {
        kRAM = 0b00000001,
        kBBR = 0b00000010,
        kFLASH = 0b00000100,
        kDEFAULT = 0b01000000,
    };

    enum {
        kVERSION_0 = 0,
        kVERSION_1 = 1,
    };

    enum {
        kVALSET_float = 2,
        kVALSET_fixed  = 3,
    };

    enum {
        kMSGOUT_RELPOSNED = 0x20910090, //Output rate of the UBX-NAV-RELPOSNED message on port USB
        kMSGOUT_PVT = 0x20910009, //Output rate of the UBX-NAV-PVT message on port USB
        kMSGOUT_POSECEF = 0x20910027, //Output rate of the UBX-NAV-POSECEF message on port USB
        kMSGOUT_VELECEF = 0x20910040, //Output rate of the UBX-NAV-VELECEF message on port USB
        kMSGOUT_RAWX = 0x209102a7, //Output rate of the UBX-RXM-RAWX message on port USB
        kMSGOUT_SFRBX = 0x20910234, //Output rate of the UBX-RXM-SFRBX message on port USB
    };

    enum { //outgoing message rates for RTCM 3x on usb type U1
        //suggested messages for stationary base
        kRTCM_1005USB = 0x209102c0, //CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
        kRTCM_1074USB = 0x20910361, //CFG-MSGOUT-RTCM_3X_TYPE1074_USB -- GPS MSM 4
        kRTCM_1084USB = 0x20910366, //CFG-MSGOUT-RTCM_3X_TYPE1084_USB -- GLONASS MSM 4
        kRTCM_1094USB = 0x2091036b, //CFG-MSGOUT-RTCM_3X_TYPE1094_USB -- Galileo MSM 4
        kRTCM_1124USB = 0x20910370, //CFG-MSGOUT-RTCM_3X_TYPE1124_USB -- Beidou MSM 4
        kRTCM_1230USB = 0x20910306, //CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2 Code-Phase Biases
        //suggested messages for mobile base
        kRTCM_4072_0USB = 0x20910301, //CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB -- UBLOX Proprietary RTCM message
        kRTCM_4072_1USB = 0x20910384, //CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB --
        kRTCM_1077USB   = 0x209102cf, //CFG-MSGOUT-RTCM_3X_TYPE1077_USB __ GPS MSM 7 (high precision)
        kRTCM_1087USB   = 0x209102d4, //CFG-MSGOUT-RTCM_3X_TYPE1087_USB __ GLONASS MSM 7 (high precision)
        kRTCM_1097USB   = 0x2091031b, //CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
        kRTCM_1127USB   = 0x209102d9, //CFG-MSGOUT-RTCM_3X_TYPE1127_USB __ Beidou MSM 7 (high precision)
        //!!!!also use RTCM_1230USB above!!!///
    };

    enum {
        kUSB_INPROT_UBX = 0x10770001, //Flag to indicate if UBX should be an input protocol on USB
        kUSB_INPROT_NMEA = 0x10770002, //Flag to indicate if NMEA should be an input protocol on USB
        kUSB_INPROT_RTCM3X = 0x10770004, //Flag to indicate if RTCM3X should be an input protocol on USB
        kUSB_OUTPROT_UBX = 0x10780001, //Flag to indicate if UBX should bean output protocol on USB
        kUSB_OUTPROT_NMEA = 0x10780002, //Flag to indicate if NMEA should bean output protocol on USB
        kUSB_OUTPROT_RTCM3X = 0x10780004, //Flag to indicate if RTCM3X should bean output protocol on USB
    };

    enum {

        kDYNMODEL = 0x20110021, //Dynamic platform model
        kDYNMODE_PORTABLE = 0,
        kDYNMODE_STATIONARY = 2,
        kDYNMODE_PEDESTRIAN = 3,
        kDYNMODE_AUTOMOTIVE = 4,
        kDYNMODE_SEA = 5,
        kDYNMODE_AIRBORNE_1G = 6,
        kDYNMODE_AIRBORNE_2G = 7,
        kDYNMODE_AIRBORNE_4G = 8,
        kDYNMODE_WRIST_WORN = 9,
        kDYNMODE_BIKE =10,
    };

    enum {
        kMSGOUT_SVIN = 0x2091008b,
        kTMODE_MODE = 0x20030001,
        kTMODE_SVIN_MIN_DUR = 0x40030010, //survey in minimum duration s
        kTMODE_SVIN_ACC_LIMIT = 0x40030011, //Survey-in position accuracy limit mm
    };

        //enum not finished, but not needed.  The rest is not needed.
    enum {
        kSIGNAL_GPS = 0x1031001f, //GPS enable
        kSIGNAL_GPS_L1 = 0x10310001, //GPS L1C/A
        kSIGNAL_GPS_L2 = 0x10310003, //GPS L2C (only on u-blox F9 platform products)
        kSIGNAL_GAL = 0x10310021, //Galileo enable
        kSIGNAL_GAL_E1 = 0x10310007, //Galileo E1
        kSIGNAL_GAL_E5B = 0x1031000a, //Galileo E5b (only on u-blox F9 platform products)
        kSIGNAL_BDS = 0x10310022, //BeiDou Enable
        kSIGNAL_BDS_B1 = 0x1031000d, //BeiDou B1I
        kSIGNAL_BDS_B2 = 0x1031000e, //BeiDou B2I
    };

    enum {
        //used for nav rate messages
        kRATE_MEAS = 0x30210001, //Nominal time between GNSS measurements (e.g. 100ms results in 10Hz measurement rate, 1000ms = 1Hz measurement rate)
        kRATE_NAV = 0x30210002, //Ratio of number of measurements to number of navigation solutions
        kRATE_TIMEREF = 0x20210003, //Time system to which measurements are aligned
    };

    enum {                      // Constants for cfg-rate-timeref/ RATE_TIMEREF
        kTIME_REF_UTC = 0,
        kTIME_REF_GPS = 1,
        kTIME_REF_GLONASS = 2,
        kTIME_REF_BUIDOU = 3,
        kTIME_REF_GALILEO = 4
    };

    uint8_t version;
    uint8_t layer;
    uint8_t reserved1[2];
    CFG_KEY_ID_t cfgDataKey;

}__attribute__((packed)) CFG_VALDEL_t;

typedef struct {
    enum {
        kTIME_REF_UTC = 0,
        kTIME_REF_GPS = 1,
        kTIME_REF_GLONASS = 2,
        kTIME_REF_BUIDOU = 3,
        kTIME_REF_GALILEO = 4
    };
    uint16_t measRate; // (ms) The elapsed time between GNSS measurements which defines the rate
    uint16_t navRate; // (cycles) The ratio between the number of measurements and the number of navigation solutions, e.g. 5 means five measurements for every navigation solution
    uint16_t timeRef; // Time system to which measurements are aligned
}__attribute__((packed)) CFG_RATE_t;

typedef struct {
        uint8_t gps_enable;
        uint8_t gps_l1;
        uint8_t gps_l2;
        uint8_t glonas_enable;
        uint8_t glonas_l1;
        uint8_t glonas_l2;
        uint8_t beidou_enable;
        uint8_t beidou_b1;
        uint8_t beidou_b2;
        uint8_t galileo_enable;
        uint8_t galileo_e1;
        uint8_t galileo_e5b;

} GNSS_CONSTELLATION_t;

typedef struct {
    uint8_t eph     : 1,
            alm     : 1,
            health  : 1,
            klob    : 1,
            pos     : 1,
            clkd    : 1,
            osc     : 1,
            utc     : 1,
            rtc     : 1,
            reserved: 6,
            aop     : 1;

}__attribute__((packed)) navBbrMask_t;

typedef struct {
    enum {
        kHARDWARE_RESET_IMMEDIATE = 0x00,
        kSOFTWARE_RESET_ALL = 0x01,
        kSOFTWARE_RESET_GNSS = 0x02,
        kHARDWARE_RESET_AFTER_SHUTDOWN = 0x04,
        kGNSS_STOP = 0x08,
        kGNSS_START = 0x09,
    };

    navBbrMask_t navBbrMask;
    uint8_t resetMode;
    uint8_t reserved;

}__attribute__((packed)) CFG_RST_t;

typedef struct  {
    enum {
        kVALIDITY_FLAGS_VALIDDATE= 0b01, // Valid UTC Date (see Time Validity section for details)
        kVALIDITY_FLAGS_VALIDTIME = 0b10, // Valid UTC Time of Day (see Time Validity section for details)
        kVALIDITY_FLAGS_FULLYRESOLVED = 0b100, // UTC Time of Day has been fully resolved (no seconds uncertainty)
    };

    enum {
        kFIX_STATUS_GNSS_FIX_OK            = 0b00000001, // Valid Fix
        kFIX_STATUS_DIFF_SOLN              = 0b00000010, // Differential Corrections were applied
        kFIX_STATUS_PSM_STATE_NOT_ACTIVE   = 0b00000000,
        kFIX_STATUS_PSM_STATE_ENABLED      = 0b00000100,
        kFIX_STATUS_PSM_STATE_ACQUISITION  = 0b00001000,
        kFIX_STATUS_PSM_STATE_TRACKING     = 0b00001100,
        kFIX_STATUS_PSM_STATE_POWER_OPTIMIZED_TRACKING   = 0b00010000,
        kFIX_STATUS_PSM_STATE_INACTIVE     = 0b00010100,
        kFIX_STATUS_HEADING_VALID          = 0b00100000,
        kFIX_STATUS_CARR_SOLN_NONE         = 0b00000000,
        kFIX_STATUS_CARR_SOLN_FLOAT        = 0b01000000,
        kFIX_STATUS_CARR_SOLN_FIXED        = 0b10000000,
    };

    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    uint16_t year; // y Year (UTC)
    uint8_t month; // month Month, range 1..12 (UTC)
    uint8_t day; // d Day of month, range 1..31 (UTC)
    uint8_t hour; // h Hour of day, range 0..23 (UTC)
    uint8_t min; // min Minute of hour, range 0..59 (UTC)
    uint8_t sec; // s Seconds of minute, range 0..60 (UTC)
    uint8_t valid; // - Validity flags (see  graphic below )
    uint32_t tAcc; // ns Time accuracy estimate (UTC)
    int32_t nano; // ns Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType; // - GNSSfix Type:
    uint8_t flags; // - Fix status flags (see  graphic below )
    uint8_t flags2; // - Additional flags (see  graphic below )
    uint8_t numSV; // - Number of satellites used in Nav Solution
    int32_t lon; // 1e-7 deg Longitude
    int32_t lat; // 1e-7 deg Latitude
    int32_t height; // mm Height above ellipsoid
    int32_t hMSL; // mm Height above mean sea level
    uint32_t hAcc; // mm Horizontal accuracy estimate
    uint32_t vAcc; // mm Vertical accuracy estimate
    int32_t velN; // mm/s NED north velocity
    int32_t velE; // mm/s NED east velocity
    int32_t velD; // mm/s NED down velocity
    int32_t gSpeed; // mm/s Ground Speed (2-D)
    int32_t headMot; // 1e-5 deg Heading of motion (2-D)
    uint32_t sAcc; // mm/s Speed accuracy estimate
    uint32_t headAcc; // 1e-5 deg Heading accuracy estimate (both motion and vehicle)
    uint16_t pDOP; // 0.01  - Position DOP
    uint8_t reserved1[6]; // - Reserved
    int32_t headVeh; // 1e-5 deg Heading of vehicle (2-D)
    int16_t magDec; // 1e-2 deg Magnetic declination
    uint16_t magAcc; // 1e-2 deg Magnetic declination accuracy
}__attribute__((packed)) NAV_PVT_t;

// NAV_RELPOSNED_t
typedef struct  {
    enum {
        kFLAGS_gnssFixOK =           0b000000001,
        kFLAGS_diffSoln =            0b000000010,
        kFLAGS_relPosValid =         0b000000100,
        kFLAGS_carrSoln_float =      0b000001000,
        kFLAGS_carrSoln_fixed =      0b000010000,
        kFLAGS_isMoving =            0b000100000,
        kFLAGS_refPosMiss =          0b001000000,
        kFLAGS_refObsMiss =          0b010000000,
        kFLAGS_relPosHeadingValid =  0b100000000,
    };

    typedef union {

        uint32_t all_flags;

        struct
        {
            bool gnssFixOk          :1;
            bool diffSoln           :1;
            bool relPosValid        :1;
            bool floatCarrSoln      :1;
            bool fixedCarrSoln      :1;
            bool isMoving           :1;
            bool refPosMiss         :1;
            bool refObsMiss         :1;
            bool relPosHeadingValid :1;
            bool relPosNormalized   :1;
            bool reserved[22];
        }__attribute__((packed));
        
    }__attribute__((packed)) RELPOSFLAGS_t;

    uint8_t version; //Message version (0x01 for this version)
    uint8_t reserved1; //Reserved
    uint16_t refStationId; //Reference Station ID. Must be in the range 0..4095
    uint32_t iTow; //GPS time of week ms of the navigation epoch. See the description of iTOW for details.
    int32_t relPosN; // North component cm of relative position vector
    int32_t relPosE; // East component cm of relative position vector
    int32_t relPosD; // Down component cm of relative position vector
    int32_t relPosLength; // Length cm of relative position vector
    int32_t relPosHeading; //Heading deg of the relative position vector. Scaled 1e-5
    uint8_t reserved2[4]; //reserved
    int8_t relPosHPN; //See Interface Description pg 157
    int8_t relPosHPE; //See Interface Description pg 157
    int8_t relPosHPD; //See Interface Description pg 157
    int8_t relPosHPLength; //See Interface Description pg 157
    uint32_t accN; //Accuracy mm of relative position North component
    uint32_t accE; //Accuracy mm of relative position East component
    uint32_t accD; //Accuracy mm of relative position Down component
    uint32_t accLength; //Accuracy mm of Length of the relative position vector
    uint32_t accHeading; //Accuracy deg of heading of the relative position vector
    uint8_t reserved3[4]; //Reserved
    RELPOSFLAGS_t flags; //See graphic in Interface Description pg 158

}__attribute__((packed)) NAV_RELPOSNED_t;



typedef struct  {

    uint8_t version; //Message version (0x01 for this version)
    uint8_t reserved1[3]; //Reserved
    uint32_t iTow; //GPS time of week ms of the navigation epoch. See the description of iTOW for details.
    uint32_t dur; //Passed survey-in observation time s
    uint32_t meanX; // Current survey-in mean position ECEF X coordinate cm
    uint32_t meanY; // Current survey-in mean position ECEF Y coordinate cm
    uint32_t meanZ; // Current survey-in mean position ECEF Z coordinate cm
    uint8_t meanXHP; //See Interface Description pg 165
    uint8_t meanYHP; //See Interface Description pg 165
    uint8_t meanZHP; //See Interface Description pg 165
    uint8_t reserved2; //Reserved
    uint32_t meanAcc; //Current survey-in mean position accuracy mm
    uint32_t obs; //number of position observations used during survey-in
    uint8_t valid; // Survey-in postion validity flag, 1=valid, otherwise 0
    uint8_t active; // survey-in in progress flag, 1 = in-progress, otherwise 0
    uint8_t reserved3[2]; //Reserved

}__attribute__((packed)) NAV_SVIN_t;

typedef struct
{
    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    int32_t ecefX; // cm ECEF X coordinate
    int32_t ecefY; // cm ECEF Y coordinate
    int32_t ecefZ; // cm ECEF Z coordinate
    uint32_t pAcc; // cm Position Accuracy Estimate
}__attribute__((packed)) NAV_POSECEF_t;

typedef struct
{
    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    int32_t ecefVX; // cm ECEF X velocity
    int32_t ecefVY; // cm ECEF Y velocity
    int32_t ecefVZ; // cm ECEF Z velocity
    uint32_t sAcc; // cm Speed Accuracy Estimate
}__attribute__((packed)) NAV_VELECEF_t;

typedef struct
{
    char swVersion[30];
    char hwVersion[30];
    char extension[10][30];
}__attribute__((packed)) MON_VER_t;

typedef struct
{
    MON_VER_t mon_ver;
    bool got_mon;
} MON_VER_DBG_t;

typedef float R4;
typedef double R8;
typedef int8_t I1;
typedef int16_t I2;
typedef int32_t I4;
typedef int64_t I8;
typedef uint8_t U1;
typedef uint16_t U2;
typedef uint32_t U4;
typedef uint8_t X1;

typedef struct
{
    R8 rcvTow;      // Measurement time of week in receiver localtime approximately aligned to the GPS timesystem.
                    // The receiver local time of week, week number and leap second information can be used to translate
                    // the time to other time systems. More information about the difference in time systems can be found
                    // in RINEX 3 documentation. For a receiver operating in GLONASS only mode, UTC time can be determined
                    // by subtracting the leapS field from GPS time regardless of whether the GPS leap seconds are valid.
    U2 week;        // GPS week number in receiver local time.
    I1 leapS;       // GPS leap seconds (GPS-UTC). This field represents the receiver's best knowledge of the leap seconds
                    // offset. A flag is given in the recStat bitfield to indicate if the leap seconds are known.
    I1 numMeas;     // number of measurements to follow
    X1 recStat;     // Receiver tracking status bitfield (see graphic below)
    U1 version;     // Message Version
    U1 reserved1[2];

    struct RawxMeas
    {
        R8 prMeas;  // Pseudorange measurement [m]. GLONASS inter-frequency channel delays are compensated with an internal calibration table.
        R8 cpMeas;  // Carrier phase measurement [cycles]. The carrierphase initial ambiguity is initialized using anapproximate value to
                    // make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code
                    // measurements in accordance with the RINEX specification.
        R4 doMeas;  // Doppler measurement (positive sign forapproaching satellites) [Hz]
        U1 gnssId;  // GNSS identifier (see Satellite Numbering for a list of identifiers)
        U1 svId;    // Satellite identifier (see Satellite Numbering)
        U1 sigId;   // New style signal identifier (see Signal Identifiers)
        U1 freqId;  // Only used for GLONASS: This is the frequencyslot + 7 (range from 0 to 13)
        U2 locktime;    // Carrier phase locktime counter (maximum 64500ms)
        U1 cno;     // Carrier-to-noise density ratio (signal strength)[dB-Hz]
        X1 prStdev; // (0.01*2^n)  Estimated pseudorange measurement standard deviation
        X1 cpStdev; // (0.004)     Estimated carrier phase measurement standard deviation (note a raw value of 0x0F indicates thevalue is invalid) (see graphic below)
        X1 doStdev; // (0.002*2^n) Estimated Doppler measurement standard deviation.
        X1 trkStat; // Tracking status bitfield
        U1 reserved;
    };
    RawxMeas meas[30];

    enum
    {
        krecStat_clkReset = 0b10,
        krecStat_leapSec = 0b01,
    };
    enum
    {
        ktrkStat_subHalfCyc = 0b1000,
        ktrkStat_HalfCyc    = 0b0100,
        ktrkStat_cpValid    = 0b0010,
        ktrkStat_prValid    = 0b0001,
    };
}__attribute__((packed)) RXM_RAWX_t;

typedef struct
{
    enum
    {
        kVERSION=0x02
    };
    
    
    uint8_t version;
    union {
        uint8_t flags;
        bool crcFailed;
    };
    uint16_t subType;
    uint16_t refStation;
    uint16_t msgType;
    
}__attribute__((packed)) RXM_RTCM_t;

typedef struct {
    public:
        typedef struct {
            U1 gnssID;
            U1 svID;
            U1 cNo;
            U1 mpathIndic;
            I4 dopplerMS;
            I4 dopplerHZ;
            U2 wholeChips;
            U2 fracChips;
            U4 codePhase;
            U1 intCodePhase;
            U1 pseuRangeRMSErr;
            U1 reserved5[2]; 
        }__attribute__((packed)) SV_INFO_t;
    
    U1 version;
    U1 reserved1[3];
    U4 gpsTOW;
    U4 gloTOW;
    U4 bdsTOW;
    U1 reserved2[4];
    U4 qzssTOW;
    U2 gpsTOWacc;
    U2 gloTOWacc;
    U2 bdsTOWacc;
    U1 reserved3[2];
    U2 qzssTOWacc;
    U1 numSV;
    U1 flags;
    U1 reserved4[8];
    SV_INFO_t sv[30];

}__attribute__((packed)) RXM_MEASX_t;

enum
{
    kGnssID_GPS = 0,
    kGnssID_SBAS = 1,
    kGnssID_Galileo = 2,
    kGnssID_Beidou = 3,
    kGnssID_Qzss = 5,
    kGnssID_Glonass = 6
};

inline int sysId(int svId)
{
    return (svId >= 1   && svId <= 32)  ? kGnssID_GPS :
           (svId >= 120 && svId <= 158) ? kGnssID_SBAS :
           (svId >= 211 && svId <= 246) ? kGnssID_Galileo :
           (svId >= 159 && svId <= 163) ? kGnssID_Beidou :
           (svId >= 33  && svId <= 64)  ? kGnssID_Beidou :
           (svId >= 193 && svId <= 197) ? kGnssID_Qzss :
           (svId >= 65  && svId <= 96)  ? kGnssID_Glonass :
           (svId == 255)                ? kGnssID_Glonass :
                                          -1;
}

enum
{
    kGPS_L1_CA,
    kGPS_L2_CL,
    kGPS_L2_CM,
    kGalileo_E1_C,
    kGalileo_E1_B,
    kGalileo_E5_BI,
    kGalileo_E5_BQ,
    kBeidou_B1I_D1,
    kBeidou_B1I_D2,
    kBeidou_B2I_D1,
    kBeidou_B2I_D2,
    kQZSS_L1_CA,
    kGlonass_L1,
    kGlonass_L2,
};

inline int sigId(int gnssId, int sigId)
{
    return (gnssId == 0 && sigId == 0) ? kGPS_L1_CA :
           (gnssId == 0 && sigId == 3) ? kGPS_L2_CL :
           (gnssId == 0 && sigId == 4) ? kGPS_L2_CM :
           (gnssId == 2 && sigId == 0) ? kGalileo_E1_C :
           (gnssId == 2 && sigId == 1) ? kGalileo_E1_B :
           (gnssId == 2 && sigId == 5) ? kGalileo_E5_BI :
           (gnssId == 2 && sigId == 6) ? kGalileo_E5_BQ :
           (gnssId == 3 && sigId == 0) ? kBeidou_B1I_D1 :
           (gnssId == 3 && sigId == 1) ? kBeidou_B1I_D2 :
           (gnssId == 3 && sigId == 2) ? kBeidou_B2I_D1 :
           (gnssId == 3 && sigId == 3) ? kBeidou_B2I_D2 :
           (gnssId == 5 && sigId == 0) ? kQZSS_L1_CA :
           (gnssId == 6 && sigId == 0) ? kGlonass_L1 :
           (gnssId == 6 && sigId == 2) ? kGlonass_L2 :
                                         -1;
}


typedef struct
{
    U1 gnssId;
    U1 svId;
    U1 reserved1;
    U1 freqId;
    U1 numWords;
    U1 chn;
    U1 version;
    U1 reserved2;
    U4 dwrd[10];
}__attribute__((packed)) RXM_SFRBX_t;

typedef union {
    uint8_t buffer[BUFFER_SIZE];
    ACK_ACK_t ACK_ACK;
    ACK_NACK_t ACK_NACK;
    CFG_MSG_t CFG_MSG;
    CFG_PRT_t CFG_PRT;
    CFG_RATE_t CFG_RATE;
    CFG_RST_t CFG_RST;
    CFG_NAV5_t CFG_NAV5;
    CFG_VALSET_t CFG_VALSET;
    CFG_VALGET_t::request_t CFG_VALGET;
    CFG_VALDEL_t CFG_VALDEL;
    NAV_PVT_t NAV_PVT;
    NAV_POSECEF_t NAV_POSECEF;
    NAV_VELECEF_t NAV_VELECEF;
    NAV_RELPOSNED_t NAV_RELPOSNED;
    RXM_RAWX_t RXM_RAWX;
    RXM_RTCM_t RXM_RTCM;
    RXM_MEASX_t RXM_MEASX;
    RXM_SFRBX_t RXM_SFRBX;
    NAV_SVIN_t NAV_SVIN;
    MON_VER_t MON_VER;
} UBX_payload_t;

typedef union
{
    uint8_t buffer[UBX_MESSAGE_BUFFER_SIZE];
    struct
    {
        uint8_t start_byte_1    : 8;
        uint8_t start_byte_2    : 8;
        uint8_t message_class   : 8;
        uint8_t message_id      : 8;
        uint16_t payload_length : 16;
        UBX_payload_t payload;
    };
} UBX_message_t;
}
#endif
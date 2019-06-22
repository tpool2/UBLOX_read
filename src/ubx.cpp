#include <chrono>
#include <stdio.h>

#include "async_comm/udp.h"

#include "UBLOX/ubx.h"

using namespace std::chrono;
using namespace std;

#define DEG2RAD (3.14159 / 180.0)
#define DBG(...) fprintf(stderr, __VA_ARGS__)
//#define DBG(...)
//Use this to see the rtcm message
//#define DBG_RTCM(...) fprintf(stderr, __VA_ARGS__)
#define DBG_RTCM(...)

UBX::UBX(async_comm::Serial& ser) :
  serial_(ser)
{
    buffer_head_ = 0;
    parse_state_ = START;
    message_class_ = 0;
    message_type_ = 0;
    length_ = 0;
    ck_a_ = 0;
    ck_b_ = 0;

    auto cb = [this](const uint8_t* buffer, size_t size)
    {
      for (int i = 0; i < size; i++)
        this->read_cb(buffer[i]);
    };
    serial_.register_receive_callback(cb);
}

//bool UBX::detect_baudrate()
//{
//  serial_.init();
//  current_baudrate_ = 0;
//  for (int i = 0; i < sizeof(baudrates)/sizeof(uint32_t); i++)
//  {
//    DBG("Trying %d baudrate\n", baudrates[i]);
//    serial_.set_baud_rate(baudrates[i]);
//    milliseconds timeout_ms(1000);
//    milliseconds start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//    milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//    while (now < start + timeout_ms)
//    {
//      now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); //millis();
//      if (got_message_)
//      {
//        DBG("Found UBX at %d baud\n", baudrates[i]);
//        current_baudrate_ = baudrates[i];
//        break;
//      }
//    }
//    if (current_baudrate_ != 0)
//      break;
//  }
//  return got_message_;
//}

bool UBX::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len)
{
  // First, calculate the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

  // Send message
  serial_.send_byte(START_BYTE_1);
  serial_.send_byte(START_BYTE_2);
  serial_.send_byte(msg_class);
  serial_.send_byte(msg_id);
  serial_.send_byte(len & 0xFF);
  serial_.send_byte((len >> 8) & 0xFF);
  serial_.send_bytes(message.buffer, len);
  serial_.send_byte(ck_a);
  serial_.send_byte(ck_b);
  return true;
}

//void UBX::set_baudrate(const uint32_t baudrate)
//{
//  DBG("Setting baudrate to %d\n", baudrate);
//  // Now that we have the right baudrate, let's configure the thing
//  memset(&out_message_, 0, sizeof(CFG_PRT_t));
//  out_message_.CFG_PRT.portID = CFG_PRT_t::PORT_USB;
//  out_message_.CFG_PRT.baudrate = baudrate;
//  out_message_.CFG_PRT.inProtoMask = CFG_PRT_t::IN_UBX | CFG_PRT_t::IN_NMEA | CFG_PRT_t::IN_RTCM | CFG_PRT_t::IN_RTCM3;
//  out_message_.CFG_PRT.outProtoMask = CFG_PRT_t::OUT_UBX | CFG_PRT_t::OUT_NMEA | CFG_PRT_t::OUT_RTCM3;
//  out_message_.CFG_PRT.mode = CFG_PRT_t::CHARLEN_8BIT | CFG_PRT_t::PARITY_NONE | CFG_PRT_t::STOP_BITS_1;
//  out_message_.CFG_PRT.flags = 0;
//  send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_PRT_t));
//  usleep(10000);
//  serial_.set_baud_rate(baudrate);
//  current_baudrate_ = baudrate;
//}

void UBX::set_dynamic_mode()
{
  memset(&out_message_, 0, sizeof(CFG_NAV5_t));
  out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
  out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
  DBG("Setting dynamic mode\n");
  send_message(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
}

void UBX::set_nav_rate(uint8_t period_ms)
{
  memset(&out_message_, 0, sizeof(CFG_RATE_t));
  out_message_.CFG_RATE.measRate = period_ms;
  out_message_.CFG_RATE.navRate = 1;
  out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_GPS;
  DBG("Setting nav rate to %d\n", period_ms);
  send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
}

void UBX::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
  memset(&out_message_, 0, sizeof(CFG_MSG_t));
  out_message_.CFG_MSG.msgClass = msg_cls;
  out_message_.CFG_MSG.msgID = msg_id;
  out_message_.CFG_MSG.rate = rate;
  DBG("Requesting %x:%x message at %d hz\n", msg_cls, msg_id, rate);
  send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
}

void UBX::read_cb(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:
        buffer_head_ = 0;
        parse_state_ = GOT_START_FRAME;
        message_class_ = 0;
        message_type_ = 0;
        length_ = 0;
        ck_a_ = 0;
        ck_b_ = 0;
        got_message_ = true;
      break;
    case GOT_START_FRAME:
      message_class_ = byte;
      parse_state_ = GOT_CLASS;
      break;
    case GOT_CLASS:
      message_type_ = byte;
      parse_state_ = GOT_MSG_ID;
      break;
    case GOT_MSG_ID:
      length_ = byte;
      parse_state_ = GOT_LENGTH1;
      break;
    case GOT_LENGTH1:
      length_ |= (uint16_t) byte << 8;
      parse_state_ = GOT_LENGTH2;
      if (length_ > BUFFER_SIZE)
      {
        num_errors_++;
        parse_state_ = START;
        std::cout << "\n the message is too big" << std::endl;
        return;
      }
      break;
    case GOT_LENGTH2:
      if (buffer_head_ < length_)
      {
        // push the byte onto the data buffer
        in_message_.buffer[buffer_head_] = byte;
        if (buffer_head_ == length_-1)
        {
          parse_state_ = GOT_PAYLOAD;
        }
        buffer_head_++;
      }
      break;
    case GOT_PAYLOAD:
      ck_a_ = byte;
      parse_state_ = GOT_CK_A;
      break;
    case GOT_CK_A:
      ck_b_ = byte;
      parse_state_ = GOT_CK_B;
      break;
    default:
      num_errors_++;
      break;
    }

    // If we have a complete packet, then try to parse it
    if (parse_state_ == GOT_CK_B)
    {
      if (decode_message())
      {
        parse_state_ = START;
      }
      else
      {
        // indicate error if it didn't work
        num_errors_++;
        DBG("\n failed to parse message\n");
        parse_state_ = START;
      }
    }
}

bool UBX::decode_message()
{
  // First, check the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
  if (ck_a != ck_a_ || ck_b != ck_b_)
    return false;

  num_messages_received_++;
  DBG("recieved message %d: ", num_messages_received_);

  // Parse the payload
  switch (message_class_)
  {
  case CLASS_ACK:
    DBG("ACK_");
    switch (message_type_)
    {
    case ACK_ACK:
      got_ack_ = true;
      DBG("ACK\n");
      break;
    case ACK_NACK:
      got_nack_ = true;
      DBG("NACK\n");
      break;
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;

  case CLASS_CFG:
    cfg_get_message_ = in_message_.CFG_VALGET;
    DBG("CFG_");
    switch (message_type_)
    {
    default:
      DBG("%d\n", message_type_);
      std::cout << "cfgData = " << (int)cfg_get_message_.cfgData << "\n";
      break;
    }
    break;

  case CLASS_NAV:
    DBG("NAV_");
    switch (message_type_)
    {
    case NAV_PVT:
      new_data_ = true;
      nav_message_ = in_message_.NAV_PVT;

      DBG("PVT\n");
      break;
    case NAV_POSECEF:
      new_data_ = true;
      pos_ecef_ = in_message_.NAV_POSECEF;
      DBG("POSECEF\n");
      break;
    case NAV_VELECEF:
      new_data_ = true;
      vel_ecef_ = in_message_.NAV_VELECEF;
      DBG("VELECEF\n");
      break;
    default:
      DBG("%d\n", message_type_);
      break;
    }
    break;
  default:
    DBG("%d_%d\n", message_class_, message_type_);
    break;
  }
  return true;
}


void UBX::calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload, uint8_t& ck_a, uint8_t& ck_b) const
{
  if (msg_cls == 5)
    volatile int debug =1;
  ck_a = ck_b = 0;

  // Add in class
  ck_a += msg_cls;
  ck_b += ck_a;

  // Id
  ck_a += msg_id;
  ck_b += ck_a;

  // Length
  ck_a += len & 0xFF;
  ck_b += ck_a;
  ck_a += (len >> 8) & 0xFF;
  ck_b += ck_a;

  // Payload
  for (int i = 0; i < len; i ++)
  {
    ck_a += payload.buffer[i];
    ck_b += ck_a;
  }
}

void UBX::turnOnRTCM()
{
    memset(&out_message_, 0, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.version = CFG_VALSET_t::VALSET_0;
    out_message_.CFG_VALSET.layer = CFG_VALSET_t::VALSET_RAM;
    out_message_.CFG_VALSET.cfgData = 0; //output every cfg epoch?
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_4072_0USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_4072_1USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1077USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1087USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1097USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1127USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.cfgDataKey = CFG_VALSET_t::RTCM_1230USB;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));

//    in_message_.CFG_VALGET.cfgData = 0; //clear value
//    memset(&out_message_, 0, sizeof(CFG_VALGET_t));
//    out_message_.CFG_VALGET.version = CFG_VALGET_t::VALGET_REQUEST;
//    out_message_.CFG_VALGET.layer = CFG_VALGET_t::VALGET_RAM;
//    out_message_.CFG_VALGET.cfgDataKey = CFG_VALGET_t::RTCM_1230USB;
//    send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

void UBX::config_rover()
{
  DBG("configuring rover \n");

  bool conf_set = 0; //use 1 to set cfg data
  if (conf_set == 1)
  {

  }

  bool conf_get = 1; //use 1 to get conf data
  if (conf_get == 1)
  {

  }

}

void UBX::config_base()
{
  DBG("configuring base \n");

  bool conf_set = 1; //use 1 to set cfg data
  if (conf_set == 1)
  {

  }

  bool conf_get = 1; //use 1 to get conf data
  if (conf_get == 1)
  {

  }
}
#ifndef _ROS_robot_msgs_BatteryStatus_h
#define _ROS_robot_msgs_BatteryStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_msgs
{

  class BatteryStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _voltage_V_type;
      _voltage_V_type voltage_V;
      typedef float _current_A_type;
      _current_A_type current_A;
      typedef float _power_W_type;
      _power_W_type power_W;
      typedef float _percent_type;
      _percent_type percent;
      typedef bool _low_warning_type;
      _low_warning_type low_warning;

    BatteryStatus():
      header(),
      voltage_V(0),
      current_A(0),
      power_W(0),
      percent(0),
      low_warning(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage_V;
      u_voltage_V.real = this->voltage_V;
      *(outbuffer + offset + 0) = (u_voltage_V.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage_V.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage_V.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage_V.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage_V);
      union {
        float real;
        uint32_t base;
      } u_current_A;
      u_current_A.real = this->current_A;
      *(outbuffer + offset + 0) = (u_current_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_A.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_A.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_A.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_A);
      union {
        float real;
        uint32_t base;
      } u_power_W;
      u_power_W.real = this->power_W;
      *(outbuffer + offset + 0) = (u_power_W.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_power_W.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_power_W.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_power_W.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->power_W);
      union {
        float real;
        uint32_t base;
      } u_percent;
      u_percent.real = this->percent;
      *(outbuffer + offset + 0) = (u_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percent);
      union {
        bool real;
        uint8_t base;
      } u_low_warning;
      u_low_warning.real = this->low_warning;
      *(outbuffer + offset + 0) = (u_low_warning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->low_warning);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage_V;
      u_voltage_V.base = 0;
      u_voltage_V.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage_V.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage_V.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage_V.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage_V = u_voltage_V.real;
      offset += sizeof(this->voltage_V);
      union {
        float real;
        uint32_t base;
      } u_current_A;
      u_current_A.base = 0;
      u_current_A.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_A.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_A.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_A.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_A = u_current_A.real;
      offset += sizeof(this->current_A);
      union {
        float real;
        uint32_t base;
      } u_power_W;
      u_power_W.base = 0;
      u_power_W.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_power_W.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_power_W.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_power_W.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->power_W = u_power_W.real;
      offset += sizeof(this->power_W);
      union {
        float real;
        uint32_t base;
      } u_percent;
      u_percent.base = 0;
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percent = u_percent.real;
      offset += sizeof(this->percent);
      union {
        bool real;
        uint8_t base;
      } u_low_warning;
      u_low_warning.base = 0;
      u_low_warning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->low_warning = u_low_warning.real;
      offset += sizeof(this->low_warning);
     return offset;
    }

    const char * getType(){ return "robot_msgs/BatteryStatus"; };
    const char * getMD5(){ return "a1b8896d25393722a5ccbb75293446a3"; };

  };

}
#endif

#ifndef _ROS_robot_msgs_EncoderData_h
#define _ROS_robot_msgs_EncoderData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robot_msgs
{

  class EncoderData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _left_ticks_type;
      _left_ticks_type left_ticks;
      typedef int32_t _right_ticks_type;
      _right_ticks_type right_ticks;

    EncoderData():
      header(),
      left_ticks(0),
      right_ticks(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_left_ticks;
      u_left_ticks.real = this->left_ticks;
      *(outbuffer + offset + 0) = (u_left_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_ticks.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_ticks.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_ticks.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_ticks);
      union {
        int32_t real;
        uint32_t base;
      } u_right_ticks;
      u_right_ticks.real = this->right_ticks;
      *(outbuffer + offset + 0) = (u_right_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_ticks.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_ticks.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_ticks.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_ticks);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_left_ticks;
      u_left_ticks.base = 0;
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_ticks = u_left_ticks.real;
      offset += sizeof(this->left_ticks);
      union {
        int32_t real;
        uint32_t base;
      } u_right_ticks;
      u_right_ticks.base = 0;
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_ticks = u_right_ticks.real;
      offset += sizeof(this->right_ticks);
     return offset;
    }

    const char * getType(){ return "robot_msgs/EncoderData"; };
    const char * getMD5(){ return "dbba4bba78e4f410b4b1b20f6a28ea4a"; };

  };

}
#endif

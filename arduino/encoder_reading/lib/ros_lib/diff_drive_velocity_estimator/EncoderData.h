#ifndef _ROS_diff_drive_velocity_estimator_EncoderData_h
#define _ROS_diff_drive_velocity_estimator_EncoderData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace diff_drive_velocity_estimator
{

  class EncoderData : public ros::Msg
  {
    public:
      typedef int32_t _countLeft_type;
      _countLeft_type countLeft;
      typedef float _countSpeedLeft_type;
      _countSpeedLeft_type countSpeedLeft;
      typedef int32_t _countRight_type;
      _countRight_type countRight;
      typedef float _countSpeedRight_type;
      _countSpeedRight_type countSpeedRight;

    EncoderData():
      countLeft(0),
      countSpeedLeft(0),
      countRight(0),
      countSpeedRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_countLeft;
      u_countLeft.real = this->countLeft;
      *(outbuffer + offset + 0) = (u_countLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_countLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_countLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_countLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->countLeft);
      union {
        float real;
        uint32_t base;
      } u_countSpeedLeft;
      u_countSpeedLeft.real = this->countSpeedLeft;
      *(outbuffer + offset + 0) = (u_countSpeedLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_countSpeedLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_countSpeedLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_countSpeedLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->countSpeedLeft);
      union {
        int32_t real;
        uint32_t base;
      } u_countRight;
      u_countRight.real = this->countRight;
      *(outbuffer + offset + 0) = (u_countRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_countRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_countRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_countRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->countRight);
      union {
        float real;
        uint32_t base;
      } u_countSpeedRight;
      u_countSpeedRight.real = this->countSpeedRight;
      *(outbuffer + offset + 0) = (u_countSpeedRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_countSpeedRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_countSpeedRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_countSpeedRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->countSpeedRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_countLeft;
      u_countLeft.base = 0;
      u_countLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_countLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_countLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_countLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->countLeft = u_countLeft.real;
      offset += sizeof(this->countLeft);
      union {
        float real;
        uint32_t base;
      } u_countSpeedLeft;
      u_countSpeedLeft.base = 0;
      u_countSpeedLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_countSpeedLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_countSpeedLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_countSpeedLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->countSpeedLeft = u_countSpeedLeft.real;
      offset += sizeof(this->countSpeedLeft);
      union {
        int32_t real;
        uint32_t base;
      } u_countRight;
      u_countRight.base = 0;
      u_countRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_countRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_countRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_countRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->countRight = u_countRight.real;
      offset += sizeof(this->countRight);
      union {
        float real;
        uint32_t base;
      } u_countSpeedRight;
      u_countSpeedRight.base = 0;
      u_countSpeedRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_countSpeedRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_countSpeedRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_countSpeedRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->countSpeedRight = u_countSpeedRight.real;
      offset += sizeof(this->countSpeedRight);
     return offset;
    }

    virtual const char * getType() override { return "diff_drive_velocity_estimator/EncoderData"; };
    virtual const char * getMD5() override { return "68f0cc4dfcb8d02fd3c9b526c6e682fc"; };

  };

}
#endif

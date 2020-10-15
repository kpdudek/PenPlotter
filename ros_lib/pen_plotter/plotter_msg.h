#ifndef _ROS_pen_plotter_plotter_msg_h
#define _ROS_pen_plotter_plotter_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pen_plotter
{

  class plotter_msg : public ros::Msg
  {
    public:
      typedef const char* _move_type_type;
      _move_type_type move_type;
      typedef int8_t _servo_angle_type;
      _servo_angle_type servo_angle;
      typedef float _setpoint_x_angle_type;
      _setpoint_x_angle_type setpoint_x_angle;
      typedef float _setpoint_y_angle_type;
      _setpoint_y_angle_type setpoint_y_angle;
      typedef int8_t _set_work_zero_type;
      _set_work_zero_type set_work_zero;

    plotter_msg():
      move_type(""),
      servo_angle(0),
      setpoint_x_angle(0),
      setpoint_y_angle(0),
      set_work_zero(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_move_type = strlen(this->move_type);
      varToArr(outbuffer + offset, length_move_type);
      offset += 4;
      memcpy(outbuffer + offset, this->move_type, length_move_type);
      offset += length_move_type;
      union {
        int8_t real;
        uint8_t base;
      } u_servo_angle;
      u_servo_angle.real = this->servo_angle;
      *(outbuffer + offset + 0) = (u_servo_angle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servo_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->setpoint_x_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->setpoint_y_angle);
      union {
        int8_t real;
        uint8_t base;
      } u_set_work_zero;
      u_set_work_zero.real = this->set_work_zero;
      *(outbuffer + offset + 0) = (u_set_work_zero.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_work_zero);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_move_type;
      arrToVar(length_move_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_move_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_move_type-1]=0;
      this->move_type = (char *)(inbuffer + offset-1);
      offset += length_move_type;
      union {
        int8_t real;
        uint8_t base;
      } u_servo_angle;
      u_servo_angle.base = 0;
      u_servo_angle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->servo_angle = u_servo_angle.real;
      offset += sizeof(this->servo_angle);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->setpoint_x_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->setpoint_y_angle));
      union {
        int8_t real;
        uint8_t base;
      } u_set_work_zero;
      u_set_work_zero.base = 0;
      u_set_work_zero.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_work_zero = u_set_work_zero.real;
      offset += sizeof(this->set_work_zero);
     return offset;
    }

    const char * getType(){ return "pen_plotter/plotter_msg"; };
    const char * getMD5(){ return "47fd586e4bf446358324813a75c96cef"; };

  };

}
#endif

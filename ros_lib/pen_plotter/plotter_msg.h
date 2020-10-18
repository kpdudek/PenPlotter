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
      typedef int32_t _servo_angle_type;
      _servo_angle_type servo_angle;
      typedef float _setpoint_x_angle_type;
      _setpoint_x_angle_type setpoint_x_angle;
      typedef float _setpoint_y_angle_type;
      _setpoint_y_angle_type setpoint_y_angle;
      typedef int32_t _max_speed_type;
      _max_speed_type max_speed;
      typedef int32_t _stop_type;
      _stop_type stop;
      typedef int32_t _set_work_zero_type;
      _set_work_zero_type set_work_zero;

    plotter_msg():
      move_type(""),
      servo_angle(0),
      setpoint_x_angle(0),
      setpoint_y_angle(0),
      max_speed(0),
      stop(0),
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
        int32_t real;
        uint32_t base;
      } u_servo_angle;
      u_servo_angle.real = this->servo_angle;
      *(outbuffer + offset + 0) = (u_servo_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->setpoint_x_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->setpoint_y_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_max_speed;
      u_max_speed.real = this->max_speed;
      *(outbuffer + offset + 0) = (u_max_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop);
      union {
        int32_t real;
        uint32_t base;
      } u_set_work_zero;
      u_set_work_zero.real = this->set_work_zero;
      *(outbuffer + offset + 0) = (u_set_work_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_work_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_work_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_work_zero.base >> (8 * 3)) & 0xFF;
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
        int32_t real;
        uint32_t base;
      } u_servo_angle;
      u_servo_angle.base = 0;
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_angle = u_servo_angle.real;
      offset += sizeof(this->servo_angle);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->setpoint_x_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->setpoint_y_angle));
      union {
        int32_t real;
        uint32_t base;
      } u_max_speed;
      u_max_speed.base = 0;
      u_max_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_speed = u_max_speed.real;
      offset += sizeof(this->max_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
      union {
        int32_t real;
        uint32_t base;
      } u_set_work_zero;
      u_set_work_zero.base = 0;
      u_set_work_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_work_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_work_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_work_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_work_zero = u_set_work_zero.real;
      offset += sizeof(this->set_work_zero);
     return offset;
    }

    const char * getType(){ return "pen_plotter/plotter_msg"; };
    const char * getMD5(){ return "a40302f72a08566c0ed01164e9eec9ca"; };

  };

}
#endif

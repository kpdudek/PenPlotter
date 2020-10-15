#ifndef _ROS_pen_plotter_plotter_feedback_h
#define _ROS_pen_plotter_plotter_feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pen_plotter
{

  class plotter_feedback : public ros::Msg
  {
    public:
      typedef float _x_angle_type;
      _x_angle_type x_angle;
      typedef float _y_angle_type;
      _y_angle_type y_angle;
      typedef int8_t _servo_angle_type;
      _servo_angle_type servo_angle;
      typedef int8_t _at_goal_type;
      _at_goal_type at_goal;

    plotter_feedback():
      x_angle(0),
      y_angle(0),
      servo_angle(0),
      at_goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->y_angle);
      union {
        int8_t real;
        uint8_t base;
      } u_servo_angle;
      u_servo_angle.real = this->servo_angle;
      *(outbuffer + offset + 0) = (u_servo_angle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servo_angle);
      union {
        int8_t real;
        uint8_t base;
      } u_at_goal;
      u_at_goal.real = this->at_goal;
      *(outbuffer + offset + 0) = (u_at_goal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->at_goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y_angle));
      union {
        int8_t real;
        uint8_t base;
      } u_servo_angle;
      u_servo_angle.base = 0;
      u_servo_angle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->servo_angle = u_servo_angle.real;
      offset += sizeof(this->servo_angle);
      union {
        int8_t real;
        uint8_t base;
      } u_at_goal;
      u_at_goal.base = 0;
      u_at_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->at_goal = u_at_goal.real;
      offset += sizeof(this->at_goal);
     return offset;
    }

    const char * getType(){ return "pen_plotter/plotter_feedback"; };
    const char * getMD5(){ return "5707d92ed4a211d4ff9607477512a5c2"; };

  };

}
#endif

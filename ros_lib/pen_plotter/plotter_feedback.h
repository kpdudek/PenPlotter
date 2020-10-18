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
      typedef int32_t _servo_angle_type;
      _servo_angle_type servo_angle;
      typedef int32_t _at_goal_type;
      _at_goal_type at_goal;
      typedef int32_t _x_speed_type;
      _x_speed_type x_speed;
      typedef int32_t _y_speed_type;
      _y_speed_type y_speed;

    plotter_feedback():
      x_angle(0),
      y_angle(0),
      servo_angle(0),
      at_goal(0),
      x_speed(0),
      y_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->y_angle);
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
      union {
        int32_t real;
        uint32_t base;
      } u_at_goal;
      u_at_goal.real = this->at_goal;
      *(outbuffer + offset + 0) = (u_at_goal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_at_goal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_at_goal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_at_goal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->at_goal);
      union {
        int32_t real;
        uint32_t base;
      } u_x_speed;
      u_x_speed.real = this->x_speed;
      *(outbuffer + offset + 0) = (u_x_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_y_speed;
      u_y_speed.real = this->y_speed;
      *(outbuffer + offset + 0) = (u_y_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y_angle));
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
      union {
        int32_t real;
        uint32_t base;
      } u_at_goal;
      u_at_goal.base = 0;
      u_at_goal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_at_goal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_at_goal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_at_goal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->at_goal = u_at_goal.real;
      offset += sizeof(this->at_goal);
      union {
        int32_t real;
        uint32_t base;
      } u_x_speed;
      u_x_speed.base = 0;
      u_x_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_speed = u_x_speed.real;
      offset += sizeof(this->x_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_y_speed;
      u_y_speed.base = 0;
      u_y_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_speed = u_y_speed.real;
      offset += sizeof(this->y_speed);
     return offset;
    }

    const char * getType(){ return "pen_plotter/plotter_feedback"; };
    const char * getMD5(){ return "f19a8337b15ad23a98ac57e3bde2084b"; };

  };

}
#endif

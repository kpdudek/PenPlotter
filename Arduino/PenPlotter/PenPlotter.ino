#include <math.h>
#include <Encoder.h>
#include <Servo.h>
#include <ros.h>
#include <pen_plotter/plotter_msg.h>
#include <pen_plotter/plotter_feedback.h>

// Good 'ol PI
float pi = 3.1415926;
float fuzzy_equal = .001;

///////////////////////////////////////////////////////////////////////////
// Pin declarations
///////////////////////////////////////////////////////////////////////////
#define x_axis_pin_CW 2
#define x_axis_pin_CCW 3

#define y_axis_pin_CW 4
#define y_axis_pin_CCW 5

#define Enc1_Pin1 27 // encoder1 CLK pin
#define Enc1_Pin2 28 // encoder1 DT pin

#define Enc2_Pin1 25 // encoder1 CLK pin
#define Enc2_Pin2 26 // encoder1 DT pin

#define blue_light 23
#define red_light 22
#define yellow_light 21
#define green_light 20

///////////////////////////////////////////////////////////////////////////
// Variable declarations
///////////////////////////////////////////////////////////////////////////
// Varables used to store time both current and 'stopwatch' times
unsigned long int t = 0, t_prev = 0;
unsigned long int motor_freq = 1000, t_old_motor = 0;
unsigned long int print_freq = 100000, t_old_print = 0;

//// Encoder pins and classes
Encoder Enc1(Enc1_Pin1, Enc1_Pin2);
long oldPosition1  = -1, newPosition1 = 0;
Encoder Enc2(Enc2_Pin1, Enc2_Pin2);
long oldPosition2  = -1, newPosition2 = 0;

///////////////////////////////////////////////////////////////////////////
// Class declarations
///////////////////////////////////////////////////////////////////////////
class Motor{
  public:
    int cw_pin, ccw_pin;
    int m_speed = 0;
    int stopped = 1;
  
    Motor(int cw_pin,int ccw_pin){
      this->cw_pin = cw_pin;
      this->ccw_pin = ccw_pin;
    }
    
    void command(int m_speed){
      this->m_speed = m_speed;
      if (this->m_speed > 0){
        this->stopped = 0;
        analogWrite(this->ccw_pin,0);
        analogWrite(this->cw_pin,this->m_speed);
      }
      else if(this->m_speed < 0){
        this->stopped = 0;
        analogWrite(this->cw_pin,0);
        analogWrite(this->ccw_pin,this->m_speed*-1);
      }
      else{
        this->stopped = 1;
        analogWrite(this->cw_pin,0);
        analogWrite(this->ccw_pin,0);
      }
    }
    void stop_now(void){
      analogWrite(this->cw_pin,0);
      analogWrite(this->ccw_pin,0);
      this->stopped = 1;
    }
};

///////////////////////////////////////////////////////////////////////////
// Object declarations
///////////////////////////////////////////////////////////////////////////
Motor x_axis_motor = Motor(x_axis_pin_CW,x_axis_pin_CCW);
float x_axis_angle = 0., prev_x_angle = 0.;
float setpoint_x = 0.0, tol_x = 0.75;
float prev_setpoint_x = 0.0;
float interp_val_x = 0.0;
float via_point_x = 0.0;
float error_via_x = 0.0, error_goal_x = 0.0;
int setpoint_reached_x = 0, via_reached_x = 0;

Motor y_axis_motor = Motor(y_axis_pin_CW,y_axis_pin_CCW);
float y_axis_angle = 0., prev_y_angle = 0.;
float setpoint_y = 0.0, tol_y = 0.75;
float prev_setpoint_y = 0.0;
float interp_val_y = 0.0;
float via_point_y = 0.0;
float error_via_y = 0.0, error_goal_y = 0.0;
int setpoint_reached_y = 0, via_reached_y = 0;

float max_x_angle = 900.0, max_y_angle = 900;
float interpolation_steps = 1.0;
float max_via_error = max_x_angle/interpolation_steps;
float kp = 0.033;

Servo pen_servo;
int servo_angle = -1, retract_angle = 15, draw_angle=0, clearance_angle = 70;

char move_type[2]; 
int loop_count = 0;
int state = 0, stop_signal = 0;
int run_once = 0;
int state_print_count = 0;
int max_speed = 150, x_speed = 85, y_speed = 85; 
int missed_packets = 0;

///////////////////////////////////////////////////////////////////////////
// ROS declarations
///////////////////////////////////////////////////////////////////////////
ros::NodeHandle nh;

void plotter_command_callback(const pen_plotter::plotter_msg &msg_data){
    setpoint_x = msg_data.setpoint_x_angle;
    setpoint_y = msg_data.setpoint_y_angle;

    strcpy(move_type,msg_data.move_type);
    
    max_speed = msg_data.max_speed;
    x_speed = max_speed;
    y_speed = max_speed;
    
    stop_signal = msg_data.stop;

    if (msg_data.servo_angle != servo_angle){
      pen_servo.write(msg_data.servo_angle);
    }
    servo_angle = msg_data.servo_angle;

    if (msg_data.set_work_zero == 1){
      x_axis_angle = 0.;
      y_axis_angle = 0.;
      setpoint_x = 0.;
      setpoint_y = 0.;
      prev_setpoint_x = 0.;
      prev_setpoint_y = 0;
      Enc1.write(0);
      Enc2.write(0);
    }
}

pen_plotter::plotter_msg cmd_msg;
pen_plotter::plotter_feedback feedback;

ros::Publisher feedback_publisher("/plotter_feedback", &feedback);
ros::Subscriber<pen_plotter::plotter_msg> plotter_subscriber("/plotter_command", &plotter_command_callback);

///////////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(x_axis_pin_CW, OUTPUT);
  pinMode(x_axis_pin_CCW, OUTPUT);

  pinMode(y_axis_pin_CW, OUTPUT);
  pinMode(y_axis_pin_CCW, OUTPUT);

  pinMode(blue_light,OUTPUT);
  pinMode(red_light,OUTPUT);
  pinMode(yellow_light,OUTPUT);
  pinMode(green_light,OUTPUT);

  pen_servo.attach(6,500,2400);
  pen_servo.write(clearance_angle);

  nh.initNode();
  nh.subscribe(plotter_subscriber);
  nh.advertise(feedback_publisher);

}

///////////////////////////////////////////////////////////////////////////
// Loop
///////////////////////////////////////////////////////////////////////////
void loop() {
  if (!Serial){
    x_axis_motor.stop_now();
    y_axis_motor.stop_now();
    pen_servo.write(clearance_angle);
    digitalWrite(blue_light,LOW);
    digitalWrite(red_light,HIGH);
    delay(500);
    digitalWrite(red_light,LOW);
    delay(500);
    missed_packets++;
    if (missed_packets == 3){
      blink_red();
      missed_packets = 0;
    }
    run_once = 0;
  }
  else if (stop_signal==0){
    /// turn on blue light
    if(run_once == 0){
      digitalWrite(red_light,LOW);
      digitalWrite(blue_light,HIGH);
      run_once = 1;
    }

    /// get loop time
    t = micros();

    /// read and update encoders
    newPosition1 = Enc1.read();
    newPosition2 = Enc2.read();
    if (newPosition1 != oldPosition1) {
        oldPosition1 = newPosition1;
        x_axis_angle = 360.*(float(newPosition1)/(12.*120.));
    }
    if (newPosition2 != oldPosition2) {
        oldPosition2 = newPosition2;
        y_axis_angle = 360.*(float(newPosition2)/(12.*120.));
    }

    /// update via_sepoints if the goal has changed
    if (setpoint_x != prev_setpoint_x){
      interp_val_x = (setpoint_x-prev_setpoint_x)/interpolation_steps;
      via_point_x = prev_setpoint_x;
      prev_setpoint_x = setpoint_x;
      via_reached_x=0;
    }
    if (setpoint_y != prev_setpoint_y){
      interp_val_y = (setpoint_y-prev_setpoint_y)/interpolation_steps;
      via_point_y = prev_setpoint_y;
      prev_setpoint_y = setpoint_y;
      via_reached_y=0;
    }

    /// set the motor speeds based on angular error from the via setpoint
    error_via_x = abs(via_point_x-x_axis_angle);
    error_via_y = abs(via_point_y-y_axis_angle);

    
    x_speed = floor(max_speed * (error_via_x*kp));
    x_speed = constrain(x_speed,50,max_speed);

    y_speed = floor(max_speed * (error_via_y*kp));
    y_speed = constrain(y_speed,50,max_speed);
    
    /// drive motors to the via setpoints
    if ((x_axis_angle < via_point_x - tol_x)&&(via_reached_x==0)){
      x_axis_motor.command(x_speed);
      setpoint_reached_x = 0;
    }
    else if ((x_axis_angle > via_point_x + tol_x)&&(via_reached_x==0)){
      x_axis_motor.command(-x_speed);
      setpoint_reached_x = 0;
    }
    else{
      x_axis_motor.stop_now();
      via_reached_x = 1;
      if((abs(setpoint_x-x_axis_angle) < tol_x)){
        setpoint_reached_x = 1;
      }
    }

    if ((y_axis_angle < via_point_y - tol_y)&&(via_reached_y==0)){
      y_axis_motor.command(y_speed);
      setpoint_reached_y = 0;
    }
    else if ((y_axis_angle > via_point_y + tol_y)&&(via_reached_y==0)){
      y_axis_motor.command(-y_speed);
      setpoint_reached_y = 0;
    }
    else{
      y_axis_motor.stop_now();
      via_reached_y = 1;
      if((abs(setpoint_y-y_axis_angle) < tol_y)){
        setpoint_reached_y = 1;
      }
    }

    /// if both motors are at their via setpoint advance to the next via setpoint
    if ((via_reached_x==1)&&(via_reached_y==1)){
      // Y via point adjustment
      if (abs(via_point_y-setpoint_y)>tol_y){
        via_point_y = via_point_y + interp_val_y;
      }
      else{
        setpoint_reached_y=1;
      }

      // X via point adjustment
      if (abs(via_point_x-setpoint_x)>tol_x){
        via_point_x = via_point_x + interp_val_x;
      }
      else{
        setpoint_reached_x=1;
      }
      via_reached_x=0;
      via_reached_y=0;
    }

    /// if the gantry has reached its goal, update the flag and hold position
    if ((setpoint_reached_y==1) && (setpoint_reached_x==1) && (state==0)){
      state = 1;
    }
    else if ((setpoint_reached_y!=1) || (setpoint_reached_x!=1)){
      state = 0;
    }

    /// Publish data at a fixed frequency
    if (t-t_old_print > print_freq){
      feedback.x_angle = x_axis_angle;
      feedback.y_angle = y_axis_angle;
      feedback.x_speed = x_speed;
      feedback.y_speed = y_speed;
      feedback.servo_angle = servo_angle;
      feedback.at_goal = state;
      feedback_publisher.publish(&feedback);
      t_old_print = t;
    }

    /// store loop variables
    prev_x_angle = x_axis_angle;
    prev_y_angle = y_axis_angle;
    t_prev = t;

    /// Spin the subscribers
    nh.spinOnce();
  }

  else{
    x_axis_motor.stop_now();
    y_axis_motor.stop_now();
    pen_servo.write(clearance_angle);
  }
}

///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////
void blink_red(void){
  for (int i=0;i<5;i++){
    digitalWrite(red_light,LOW);
    delay(100);
    digitalWrite(red_light,HIGH);
    delay(100);
  }
}

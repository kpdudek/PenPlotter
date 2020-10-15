#include <math.h>
#include <Encoder.h>
#include <Servo.h>
#include <ros.h>
#include <pen_plotter/plotter_msg.h>
#include <pen_plotter/plotter_feedback.h>

// Good 'ol PI
float pi = 3.1415926;

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
unsigned long int t = 0;
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
float x_axis_angle = 0.;
float setpoint_x = 0.0, tol_x = 1;
int setpoint_reached_x = 0;

Motor y_axis_motor = Motor(y_axis_pin_CW,y_axis_pin_CCW);
float y_axis_angle = 0.;
float setpoint_y = 0.0, tol_y = 1;
int setpoint_reached_y = 0;

int loop_count = 0;
Servo pen_servo;
int servo_angle = -1, retract_angle = 15, draw_angle=0;

///////////////////////////////////////////////////////////////////////////
// ROS declarations
///////////////////////////////////////////////////////////////////////////
ros::NodeHandle nh;

void plotter_command_callback(const pen_plotter::plotter_msg &msg_data){
    setpoint_x = msg_data.setpoint_x_angle;
    setpoint_y = msg_data.setpoint_y_angle;

    if (msg_data.servo_angle != servo_angle){
      pen_servo.write(msg_data.servo_angle);
    }
    servo_angle = msg_data.servo_angle;

    if (msg_data.set_work_zero == 1){
      x_axis_angle = 0.;
      y_axis_angle = 0.;
      setpoint_x = 0.;
      setpoint_y = 0.;
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
  pen_servo.write(retract_angle);

  nh.initNode();
  nh.subscribe(plotter_subscriber);
  nh.advertise(feedback_publisher);

}
int missed_packets = 0;

///////////////////////////////////////////////////////////////////////////
// Loop
///////////////////////////////////////////////////////////////////////////

int state = 0;
int run_once = 0;
int state_print_count = 0;
int x_speed = 150, y_speed = 130; 

void blink_red(void){
  for (int i=0;i<10;i++){
    digitalWrite(red_light,LOW);
    delay(100);
    digitalWrite(red_light,HIGH);
    delay(100);
  }
}
void loop() {
  if (!Serial){
    x_axis_motor.stop_now();
    y_axis_motor.stop_now();
    pen_servo.write(retract_angle);
    digitalWrite(blue_light,LOW);
    digitalWrite(red_light,HIGH);
    delay(1500);
    digitalWrite(red_light,LOW);
    delay(1500);
    missed_packets++;
    if (missed_packets == 3){
      blink_red();
    }
    run_once = 0;
  }
  else{
    t = micros();
    newPosition1 = Enc1.read();
    newPosition2 = Enc2.read();

    if(run_once == 0){
      digitalWrite(red_light,LOW);
      digitalWrite(blue_light,HIGH);
      run_once = 1;
    }
    
    if (newPosition1 != oldPosition1) {
        oldPosition1 = newPosition1;
        x_axis_angle = 360.*(float(newPosition1)/(12.*120.));
    }
  
    if (newPosition2 != oldPosition2) {
        oldPosition2 = newPosition2;
        y_axis_angle = 360.*(float(newPosition2)/(12.*120.));
    }
  
    if (t-t_old_motor > motor_freq){
      if (x_axis_angle < setpoint_x - tol_x){
        x_axis_motor.command(x_speed);
        setpoint_reached_x = 0;
      }
      else if (x_axis_angle > setpoint_x + tol_x){
        x_axis_motor.command(-x_speed);
        setpoint_reached_x = 0;
      }
      else{
        x_axis_motor.stop_now();
        setpoint_reached_x = 1;
      }
  
      if (y_axis_angle < setpoint_y - tol_y){
        y_axis_motor.command(y_speed);
        setpoint_reached_y = 0;
      }
      else if (y_axis_angle > setpoint_y + tol_y){
        y_axis_motor.command(-y_speed);
        setpoint_reached_y = 0;
      }
      else{
        y_axis_motor.stop_now();
        setpoint_reached_y = 1;
      }
      t_old_motor = t;
    }

    if ((setpoint_reached_y==1) && (setpoint_reached_x==1) && (state==0)){
      state = 1;
    }
    else if ((setpoint_reached_y!=1) || (setpoint_reached_x!=1)){
      state = 0;
    }

    if (t-t_old_print > print_freq){
      feedback.x_angle = x_axis_angle;
      feedback.y_angle = y_axis_angle;
      feedback.servo_angle = servo_angle;
      feedback.at_goal = state;
      feedback_publisher.publish(&feedback);
      t_old_print = t;
    }
    
    nh.spinOnce();
  }
}

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "cheyou_toy_car/MsgDriverCmd.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

#define KEYCODE_q 0x71
#define KEYQODE_Q 0x51    // Q and q for direction
#define KEYCODE_SP 0x20   // for stop the car
#define KEYCODE_ENTER 0x0d //for start the car 

#define STOP_THROTTLE 10.0
#define MAX_THROTTLE  15.0
#define START_THROTTLE 13.0
#define MAX_STEER 10.0
#define NEAR_ZERO 5.0


class TeleopToyCar
{
public:
  TeleopToyCar();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double throttle_, steer_, t_step, s_step;
  bool move_foward,from_stop_to_start;
  ros::Publisher car_cmd_pub_;
  
};

TeleopToyCar::TeleopToyCar():
  throttle_(0),
  steer_(0),
  t_step(0.1),
  s_step(0.1),
  move_forward(true),
  from_stop_to_start(true)
{

  car_cmd_pub_ = nh_.advertise<cheyou_toy_car::MsgDriverCmd>("/car_cmd", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_toycar");
  TeleopToyCar teleop_toycar;

  signal(SIGINT,quit);

  teleop_toycar.keyLoop();
  
  return(0);
}


void TeleopToyCar::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to drive the toy car.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

  

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        steer_ -= s_step;
        steer_ = std::max(steer_,-MAX_STEER);
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        steer_ += s_step;
        steer_ = std::min(steer_,MAX_STEER);
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        
        if (throttle_ < - NEAR_ZERO){
          throttle_ += t_step;                
          throttle_ = std::min(throttle_,-STOP_THROTTLE);
        }
        else{
          throttle_ += t_step; 
          throttle_ = std::min(throttle_,MAX_THROTTLE); //限制一个正向的最大的油门值  
        }   
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        if (throttle_ > NEAR_ZERO){
          throttle_ -= t_step;
          throttle_ =std::max(throttle_,STOP_THROTTLE);  //Limit the lowest throttle to avoid car stop
        }
        else{
          throttle_ -= t_step;
          throttle_ = std::max(throttle_,-MAX_THROTTLE); //Limit the highest throttle to avoid car run too fast   
        }
        dirty = true;
        break;
      case KEYCODE_SP:
        ROS_DEBUG("SPACE BAR");
        throttle_ = 0.0;
        puts("throttle will be zero, but can not assure the car is actually stoped!!!")
        dirty = true;
        break;
      case KEYCODE_ENTER:
        ROS_DEBUG("ENTER KEY");
        //we need a big throttle to star the car
        if(move_forward)
          throttle_ = START_THROTTLE;
        else
          throttle_ =-START_THROTTLE;
        break;
      case KEYCODE_q:
      case KEYCODE_Q:
        move_forward = ! move_forward;
        break;

    }
   

    cheyou_toy_car::MsgDriverCmd drive_cmd;
    drive_cmd.throttle_pos = throttle_;
    drive_cmd.steer_pos = steer_;

    if(dirty ==true)
    {
      car_cmd_pub_.publish(drive_cmd);    
      dirty=false;
    }
  }


  return;
}




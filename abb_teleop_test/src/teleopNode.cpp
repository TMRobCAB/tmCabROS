#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <linux/input.h>
#include <termios.h>

// teleop ABB class definition

class TeleopABB
{
public:
  TeleopABB();
  ros::Publisher pose_pub_;

private:

  ros::NodeHandle nh_;
  int linear_, angular_;
  double l_scale_, a_scale_;
};

// Teleop ABB constructor

TeleopABB::TeleopABB():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vrep/abbPoseDes", 1);

}
int kfd = 0;
struct termios cooked, raw;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_abb");
  TeleopABB teleop_abb;

  geometry_msgs::PoseStamped poseDes;

  char c;
  bool dirty;
  double abs_z=0.5;
  double abs_x=0.4;

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
  puts("Use arrow keys to move the turtle.");


  while(ros::ok()){
  //1) read keyboard
   // get the next event from the keyboard
	       if(read(kfd, &c, 1) < 0)
	       {
	         perror("read():");
	         exit(-1);
	       }

	       ROS_DEBUG("value: 0x%02X\n", c);

	       switch(c)
	       {
	       	   case 'w':
	           ROS_INFO("up");
	           abs_z+= 0.02;
	           dirty = true;
	           break;
	         case 's':
	        	 ROS_INFO("down");
	        	abs_z+= -0.01;

	           dirty = true;
	           break;
	         case 'a':
	        	 ROS_INFO("left");
	        	 abs_x+= 0.01;
	           dirty = true;
	           break;
	         case 'd':
	        	 ROS_INFO("right");
	        	 abs_x+= -0.01;
	           dirty = true;
	           break;
	       }

  //2) create geometry_msgs/PoseStamped message

	       poseDes.pose.position.x=abs_x;
	       poseDes.pose.position.z=abs_z;
	       if(dirty == true)
	       {
	    	   teleop_abb.pose_pub_.publish(poseDes);
	    	   dirty=false;
	       }

	  ros::Duration(0.1).sleep();
  }
  //ros::spin();
}

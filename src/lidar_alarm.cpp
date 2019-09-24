// lab 2 by Jieyu Ren (jxr477), based on examples "stdr_open_loop_commander" and "lidar_alarm" from the textbook: A Systematic Approach to Learning Robot Programming with ROS

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

const double MIN_SAFE_DISTANCE = 0.8; // set alarm if anything is within 0.8m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

double upper = 0.0; // upper bound
double lower = 0.0; // lower bound
float dist = 3.0;  // distance scanned by pings


ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::Publisher twist_commander;

// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {

double sample_dt = 0.01; // specify a sample period of 10 ms
double speed = 0.15; // 0.15 m / s speed command
double yaw_rate = 0.6; // 0.6 rad / sec yaw rate command
double time = 2;
    geometry_msgs::Twist twist_cmd;
    twist_cmd.linear.x =0.0;
    twist_cmd.linear.y =0.0;
    twist_cmd.linear.z =0.0;
    twist_cmd.angular.x =0.0;
    twist_cmd.angular.y =0.0;
    twist_cmd.angular.z =0.0;
    ros::Rate loop_timer(1/sample_dt);
    double timer =0.0;


     if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
    }



   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
    if (timer > 0.0){
	ROS_DEBUG("Timer is not reset!");
	}

    twist_cmd.linear.x = speed; // command to move forward
	for (int i =0; i <10; i ++) {
	twist_commander.publish(twist_cmd);
	loop_timer.sleep();
	}

   // scan up to 1.5 rad (around 90 degrees) on left and right side, 
   // and calculate the upper and lower bound ping index
   upper = (int) ping_index_ + 1.5 / angle_increment_; 
   lower = (int) ping_index_ - 1.5 / angle_increment_;

   laser_alarm_=false; // reset the alarm eveytime a new call comes in

   for (int i = lower; i < upper; i++) { // loop from lower bound ping to upper bound ping
     
     dist = laser_scan.ranges[i]; // calculate the wall distance at current ping

     if (dist < MIN_SAFE_DISTANCE) { // alarm if current ping distance is smaller than safety
         ROS_INFO("DANGER!");
         laser_alarm_=true; 
         twist_cmd.linear.x = 0.0;    // command to stop
	for (int i =0; i <10; i ++) {
	twist_commander.publish(twist_cmd);
	loop_timer.sleep();
	}
	timer = 0.0;
         twist_cmd.angular.z = yaw_rate; // start spinning in place
        while (timer < time) {
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
        }
	twist_cmd.angular.z = 0.0; // stop spinning
	twist_cmd.linear.x = speed; // and move forward again
        timer =0.0; // reset the timer
  	while (timer < time) {
	twist_commander.publish(twist_cmd);
	timer += sample_dt;
	loop_timer.sleep();
	}
	twist_cmd.linear.x = 0.0;
	twist_cmd.angular.z = 0.0;
	for (int i =0; i <10; i ++) {
	twist_commander.publish(twist_cmd);
	loop_timer.sleep();
	}
	timer = 0.0;
	if (twist_cmd.linear.x > 0.0)
	ROS_DEBUG("Robot didn't stop!");
     }
   }


   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);  
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    
    std_msgs::String topic_name;
    std::stringstream ss;
    ss << "/robot0/laser_1";
    topic_name.data = ss.str();

    int opt;
    while ((opt = getopt(argc, (argv), "n:")) != -1) {
      switch (opt) {
        case 'n':
	  topic_name.data = optarg;
	  break;
	default:
	  printf("The -%c is not a recognized parameter\n", opt);
	  break;
	}
    }
    twist_commander = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;

    ros::Subscriber lidar_subscriber = nh.subscribe(topic_name.data.c_str(), 1, &laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival

    return 0; // should never get here, unless roscore dies
}

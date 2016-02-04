#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

/**Set to true for left, false for right.*/
const bool chirality_left = true;

/**Keep #set_point_dist_from_wall (meters) far away from the wall, +/-
   #band_width (meters).*/
const double set_point_dist_from_wall = 0.5;

/**Keep #set_point_dist_from_wall (meters) far away from the wall, +/-
   #band_width (meters).*/
const double band_width = 0.1;

/**Operate at this frequency.*/
const double rate_hz = 100.0;

/** Making these static for ease of access. */
geometry_msgs::Twist twist_cmd;
double current_dist_from_wall = -1.0;

ros::Rate*       loop_timer;
ros::Publisher*  pub; //send Twist commands here
ros::Subscriber* sub; //receive LIDAR scans here

bool g_lidar_alarm_l = false;
bool g_lidar_alarm_r = false;

/**Spin this fast (rad/s) (must be positive).*/
const double spin_rate = M_PI * 0.75;

/**Go forwards this fast (m/s).*/
const double speed = 0.75;

////////////////////////////////////////////////////////////////////////////////

/**
 * Ensures that #twist_cmd is zeroed-out.  Good for starting and
 * stopping the node (sometimes the early/late packets get lost).
 */
void blank_out_twist(void) {
	twist_cmd.linear.x  = 0.0;
	twist_cmd.linear.y  = 0.0;
	twist_cmd.linear.z  = 0.0;
	twist_cmd.angular.x = 0.0;
	twist_cmd.angular.y = 0.0;
	twist_cmd.angular.z = 0.0;
}

/**
 * Send #twist_cmd at a rate of #rate_hz for one second.
 */
void spray_n_twist_cmds(int n) {
	while (n-->0) {
		pub->publish(twist_cmd);
		loop_timer->sleep();
	}
}

////////////////////////////////////////////////////////////////////////////////

void alarmCallbackL(const std_msgs::Bool& alarm_msg) {
	// make the alarm status global, so main() can use it
	g_lidar_alarm_l = alarm_msg.data;
	// if (g_lidar_alarm_l) {
	// 	ROS_INFO("LIDAR alarm received!");
	// }
}

void alarmCallbackR(const std_msgs::Bool& alarm_msg) {
	// make the alarm status global, so main() can use it
	g_lidar_alarm_r = alarm_msg.data;
	// if (g_lidar_alarm_r) {
	// 	ROS_INFO("LIDAR alarm received!");
	// }
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	const int known_right_angle_index = 83;
	current_dist_from_wall =
	  laser_scan.ranges[chirality_left ?
	                    667-known_right_angle_index :
	                    known_right_angle_index];
	ROS_INFO("This far from wall: %f", current_dist_from_wall);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @return positive for too far, negative for too close, 0 for level.
 */
double check_if_in_dead_band(void) {
	double max = set_point_dist_from_wall + band_width;
	double min = set_point_dist_from_wall - band_width;
	if (max < current_dist_from_wall) {
		return 1.0;  //too far
	} else if (min > current_dist_from_wall ) {
		return -1.0; //too close
	} else {
		return 0.0;  //good enough
	}
  // double out = (current_dist_from_wall - set_point_dist_from_wall) * 5.0;
  // if (out >  1.0) out = 1.0;
  // if (out < -1.0) out = -1.0;
  // return out;
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Node to send Twist commands to the Simple 2-Dimensional Robot
 * Simulator via cmd_vel
 */
int main(int argc, char** argv) {
	// two lines to create a publisher object that can talk to ROS
	ros::init(argc, argv, "commander");
	ros::NodeHandle n;

	//help limit our execution speed
	ros::Rate aloop_timer(rate_hz);
	loop_timer = &aloop_timer;

	// Connect to UDP-style pipes/firehoses
	ros::Publisher apub =
	  n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	pub = &apub;
	ros::Subscriber asub =
	  n.subscribe("/robot0/laser_0", 1, laserCallback);
	sub = &asub;

	ros::Subscriber alarm_subscriber_l
	  = n.subscribe("lidar_alarm_l", 1, alarmCallbackL);

	ros::Subscriber alarm_subscriber_r
	  = n.subscribe("lidar_alarm_r", 1, alarmCallbackR);

	//"clear the pipes"
	blank_out_twist();
	spray_n_twist_cmds(10);

	while(ros::ok()) {
		// spin towards wall by default (first line), but
		// allow alarm and distance to override (second line)
		twist_cmd.angular.z = spin_rate * (chirality_left ? 1 : -1);
		if (chirality_left ? g_lidar_alarm_l : g_lidar_alarm_r) {
		  //spin in place towards the wall
		  twist_cmd.angular.z *= -1.0;
		  twist_cmd.linear.x = 0;
		} else {
		  //arc in correct direction
		  twist_cmd.angular.z *= check_if_in_dead_band();
		  twist_cmd.linear.x = speed;
		}

		// write out changes
		pub->publish(twist_cmd);
		ros::spinOnce();
		loop_timer->sleep();
	}
	//done commanding the robot; node runs to completion
}

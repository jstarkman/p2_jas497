#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message

const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot
const double ROBOT_RADIUS = 0.2 + 0.1;

// these values to be set within the laser callback
float ping_dist_in_front_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
bool first_pass = true;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_ = false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

/**
 * Reminder: `$ rosmsg show sensor_msgs/LaserScan` to see the struct.
 */
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	if (first_pass)  {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;
		first_pass = false;
	}

	laser_alarm_ = false;
	int i = laser_scan.ranges.size();
	double theta_i = angle_min_;
	double x, y, r = 0.0;
	while (i-->0){
		r = laser_scan.ranges[i];
		theta_i = angle_min_ + angle_increment_ * i;
		x = r * cos(theta_i);
		y = r * sin(theta_i);
		if ( (0 < x && x < MIN_SAFE_DISTANCE) && (-ROBOT_RADIUS < y && y < ROBOT_RADIUS) ){
			laser_alarm_ = true;
			ROS_WARN("Detected an object in front of me.  i=%d, X=%f, Y=%f", i, x, y);
			break; //no need to keep looking
		}
		// theta_i += angle_increment_;
	}

	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = laser_alarm_;
	lidar_alarm_publisher_.publish(lidar_alarm_msg);
	// std_msgs::Float32 lidar_dist_msg;
	// lidar_dist_msg.data = ping_dist_in_front_;
	// lidar_dist_publisher_.publish(lidar_dist_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;
	//create a Subscriber object and have it subscribe to the lidar topic
	ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
	lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
	ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);
	lidar_dist_publisher_ = pub2;
	ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}


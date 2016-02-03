#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h> // boolean message

/** Set alarm if anything is within 0.5m of the front of robot, or too
    close to the sides. */
const double MIN_SAFE_DISTANCE = 0.5;
const double ROBOT_RADIUS = 0.2 + 0.05;

/** these values to be set within the laser callback */
bool first_pass = true;
// double angle_min_ = 0.0;
// double angle_max_ = 0.0;
// double angle_increment_ = 0.0;
// double range_min_ = 0.0;
// double range_max_ = 0.0;
bool laser_alarm_ = false;
/** Precomputed for speed. */
double* angle_cache_sin;
double* angle_cache_cos;

ros::Publisher lidar_alarm_publisher_;

/**
 * Reminder: `$ rosmsg show sensor_msgs/LaserScan` to see the struct.
 */
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	std_msgs::Bool lidar_alarm_msg;
	int i = laser_scan.ranges.size();
	double
	  angle_min_,
	  angle_increment_,
	  theta_i,
	  x,
	  y,
	  r
	  = 0.0;

	if (first_pass) {
		//easier to use globals than dereference every time
		angle_min_ = laser_scan.angle_min;
		// angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		// range_min_ = laser_scan.range_min;
		// range_max_ = laser_scan.range_max;
		angle_cache_sin = new double[i];
		angle_cache_cos = new double[i];
		while (i-->0) {
			// could use x,y,r as registers instead, but
			// this is more legible
			theta_i = angle_min_ + angle_increment_ * i;
			angle_cache_cos[i] = cos(theta_i);
			angle_cache_sin[i] = sin(theta_i);
		}
		i = laser_scan.ranges.size();
		first_pass = false;
	}
	laser_alarm_ = false;

	while (i-->0) {
		r = laser_scan.ranges[i];
		x = r * angle_cache_cos[i];
		y = r * angle_cache_sin[i];
		if ( (0 < x && x < MIN_SAFE_DISTANCE) &&
		     (-ROBOT_RADIUS < y && y < ROBOT_RADIUS) ){
			laser_alarm_ = true;
			ROS_WARN("Detected an object in front of me.");
			break; //no need to keep looking
		}
	}
	lidar_alarm_msg.data = laser_alarm_;
	lidar_alarm_publisher_.publish(lidar_alarm_msg);
}

/**
 * Removed now-useless distance-in-front thing.
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;
	//create a Subscriber object and have it subscribe to the
	//lidar topic (made global so callback can use it)
	ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
	lidar_alarm_publisher_ = pub;
	ros::Subscriber lidar_subscriber =
	  nh.subscribe("robot0/laser_0", 1, laserCallback);

	// Nothing else to do here.
	ros::spin();
	return 0;
}

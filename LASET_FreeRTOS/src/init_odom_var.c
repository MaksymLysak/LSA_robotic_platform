#include "odometry_msg.h"

Odometry init_odom_var(void){
	Odometry odom;
	odom.pose.orientation.z=0;
	odom.pose.orientation.w=0;
	odom.pose.position.x=0;
	odom.pose.position.y=0;
	odom.twist.linear.x=0;
	odom.twist.angular.z=0;
	return odom;
}

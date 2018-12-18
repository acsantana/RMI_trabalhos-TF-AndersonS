/*
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Main ROS node for Occupancy Grid generation
*
*/

#include <iostream>										// cout

#include "ros/ros.h"									// ROS
#include "sensor_msgs/LaserScan.h"						// Hokuyo laser msgs
#include "geometry_msgs/Twist.h"						// Twist - message for motion
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <fstream>

//configurations
#include "../include/Constants.h"

//algorithms and data structures
#include "../include/OccupancyGrid.h"
#include "../include/Himm.h"

using namespace geometry_msgs;

//Prototypes
void hokuyoCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void keyboardCallback(const geometry_msgs::Twist& twist);
void move(double, double , bool);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
double toEulerAngle(double x, double y, double z, double w);

//Global Variables
bool HOKUYO_DISABLE = false;
ros::Publisher pub_velocity;

//global occupancy grid, hmmi and potential fields
OccupancyGrid* 		_occupancy_grid;
Himm*               _himm;
														
//global position vector (updated in odom callback)
geometry_msgs::Pose2D _pos;

/**
 * Responsable for robot motion control and mapping filling.
 * @author Anderson Domingues and Darlan Alves Jurak
 */
int main(int argc,char **argv)
{
	ros::init(argc,argv,"OccupancyGridNode");
	ros::NodeHandle n;

	//subscribe to Hokuyo node (laser)
	ros::Subscriber sub_scan 		= n.subscribe("/scan", 1, hokuyoCallback);

	//subscribe to Odom node (odometry and movement)
	ros::Subscriber sub_odom 		= n.subscribe("/odom", 1, odomCallback);

	//get keyboard click
	ros::Subscriber sub_keyboard 	= n.subscribe("/keyboardClick", 1, keyboardCallback);

	// Defines robot motion publisher
	pub_velocity = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);	

	//instantiates a new occupancy grid, hmmi and potential fields algorithms
 	_occupancy_grid 	= new OccupancyGrid();
	_himm 				= new Himm(_occupancy_grid);
	// _potential_fields 	= new PotentialFields(_occupancy_grid);

	// Let ROS take over
	ros::spin();

	return 0;
}

/*using namespace geometry_msgs;
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Hokuyo range info "print"
*
*/
void hokuyoCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double reading; 

	// ROS_INFO("min_angle [%f] max_angle [%f]", scan->angle_min, scan->angle_max);
	// ROS_INFO("angle_increment [%f]", scan->angle_increment);
	// ROS_INFO("range_min [%f] range_max [%f]", scan->range_min, scan->range_max);

	if (HOKUYO_DISABLE == false){
	
		//update occupancy grid for all ranges
		double  i = HOKUYO_ANGLE_MIN;
		int     j = 0;

		while(i <= HOKUYO_ANGLE_MAX){

			if((j % HOKUYO_LASER_SKIP) == 0){

				reading = scan->ranges[j];

				if(reading <= HOKUYO_RANGE_MAX && reading >= HOKUYO_RANGE_MIN && !std::isnan(reading)){
					_himm->UpdateLocation(_pos, reading, i);
				}
			}

			i += HOKUYO_ANGLE_INC;
			j++;
		}
	}
    //update occupancy grid for all ranges
	// double i = HOKUYO_ANGLE_MIN;
	// int j = 0;

	// while(i < HOKUYO_ANGLE_MAX){
	 	
	// 	reading = scan->ranges[j];

	// 	if(reading <= HOKUYO_RANGE_MAX && reading >= HOKUYO_RANGE_MIN && !std::isnan(reading)){
	// 		_himm->UpdateLocation(_pos, reading, i);
	// 	}

	// 	i += HOKUYO_ANGLE_INC;
	// 	j++;
	// }

	//save occupancy grid to file
	// _himm->ToFile("/home/lsa/Desktop/_himm.html");
	_himm->ToFile("/home/darlan/Darlan/_himm.html");

	//recalculate routes using potential fields and print to file
	// _potential_fields->UpdateRoutes();
	// _potential_fields->ToFile("/home/darlan/Darlan/filename.html");
}

/*
*
*	@Author: 		Darlan Alves Jurak
*	@Brief: 		Basic motion function.
* 	@Description:	Forward and backward motion based on desired velocity and distance.
*
*/
void move(double speed, double angle, double distance, bool isForward){

	// Motion message (angular and linear x, y and z )
	geometry_msgs::Twist vel_msg;

	// Set linear velocity in the x-axis
	if(isForward){

		vel_msg.linear.x = abs(speed);

	}else{

		vel_msg.linear.x = -abs(speed);

	}

	vel_msg.angular.z = angle;

	// Anulates velocity in "y" and "z" axis 
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	double t0 = ros::Time::now().toSec();		// Base time for comparison
	double current_distance = 0;				
	ros::Rate loop_rate(10);
	do{

		pub_velocity.publish(vel_msg);			// Moves robot
		double t1 = ros::Time::now().toSec();	// Get current time
		current_distance = speed * (t1 - t0);	// Updates travelled distance estimation

		loop_rate.sleep();

	}while( current_distance < distance);

	vel_msg.linear.x = 0;
	pub_velocity.publish(vel_msg);

}

/*
*	@Author: 		Anderson Domingues
*	@Brief: 		
* 	@Description:	Responsable for update current location (from odometry)
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	//update location
	_pos.x		= msg->pose.pose.position.x;
	_pos.y		= msg->pose.pose.position.y;

	//update angle
	_pos.theta 	= toEulerAngle(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w //already in radians
	);

	//ROS_INFO("position=: [%f] [%f] ([%f])", _pos.x, _pos.y, _pos.theta);
}

// 
/**
 * Function to convert from quarternion to euler angle 
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
 * @param x Quarternion.x
 * @param y Quarternion.y
 * @param z Quarternion.z
 * @param w Quarternion.w
 * @param yaw Pointer to the variable to be written with the eulerian 
 * value associated with the given quarternion */
double toEulerAngle(double x, double y, double z, double w){
	double siny = +2.0 * 		(w * z + x * y);
	double cosy = +1.0 - 2.0 * 	(y * y + z * z);  
	return atan2(siny, cosy); 
}

void keyboardCallback(const geometry_msgs::Twist& twist){

	uint32_t option = twist.linear.x;
	std::ofstream of;
	Vector2D vec;

	switch (option)
	{
		case 12: //salva mapa
			of.open("/home/darlan/Desktop/map.map", std::ofstream::trunc);
			of << _occupancy_grid->ToMap();
			of.close();
			break;

		case 13: //load map
			_occupancy_grid->LoadMap("/home/darlan/Desktop/map.map");
			break;
	
		case 14: //disable hokuyo
			HOKUYO_DISABLE = true;
			break;

		case 15: //enable hokuyo
			HOKUYO_DISABLE = false;	
			break;


		case 16: // potencial field
			vec.x = OG_SEC_W /2;
			vec.y = OG_SEC_H /2;
			_occupancy_grid->SetGoal(vec);
			_occupancy_grid->UpdatePotentialFields();
			break;

		case 17: // potencial field
			vec.x = twist.angular.x;
			vec.y = twist.angular.y;
			_occupancy_grid->SetGoal(vec);
			_occupancy_grid->UpdatePotentialFields();
			break;

		case 18: // potencial field
			_occupancy_grid->PathPlanning(_pos);
			break;

		case 20: // 
			_occupancy_grid->FollowPath();
			break;
	}
}

std::queue<Vector2D> OccupancyGrid::GetPath(){

	return this->path;

}

void OccupancyGrid::FollowPath(){

	std::queue<Vector2D> path;
	path = _occupancy_grid->GetPath();

	Vector2D nextPos;
	geometry_msgs::Twist vel_msg;
	double hipotenusa = 0;
	double angle = 0;

	for(int i = 0; i < path.size(); i++){
		
		nextPos 	= path.front();
		hipotenusa 	= sqrt((pow(nextPos.x, 2) + pow(nextPos.y, 2)));
		angle 		= acos(nextPos.x/hipotenusa);

		move(1, _pos.theta + angle, 0.1, true);
		
		path.pop();
	}

}
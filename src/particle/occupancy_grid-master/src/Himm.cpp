#include "../include/Himm.h"
#include "ros/ros.h"

/**
 * Ctor.
 * @grid Reference to the grid in which the hmmi
 * will read and write values.
 */
Himm::Himm(OccupancyGrid* grid){
	this->_grid = grid;
}

void Himm::ToFile(std::string filename){
	this->_grid->ToFile(filename);
}

Himm::~Himm(){}

/**
 * Set a value to the grid according to the position of the robot
 * @param pose A Pose2D struct representing the position and rotation of the robot
 * @param dist The distance captured by the sensor
 * @param theta The inclination of the laser ray during the reading in degrees
 * @returns The value written to the cell
 */
OGCellType Himm::UpdateLocation(Pose2D pose, OGCellType dist, OGCellType thetaRay){

	// angle difference between robot angle and hokuyo ray angle
	double corrected_angle = pose.theta + thetaRay;

	// Final position of the hokuyo ray.
	Vector2D vecLaser;
	vecLaser.x = (cos(corrected_angle) * dist + pose.x); // Global x pos of the ray
	vecLaser.y = (sin(corrected_angle) * dist + pose.y); // Global y pos of the ray

	//inicio da limpeza
	Vector2D md;
	md.x = round(pose.x * UNIT_FIX);
	md.y = round(pose.y * UNIT_FIX);

	Vector2D target;
	target.x = round(vecLaser.x * UNIT_FIX);
	target.y = round(vecLaser.y * UNIT_FIX);

	while((md.x != target.x) || (md.y != target.y)){

		_grid->Set(
				md.x, 
				md.y, 
				-1);

		// ROS_INFO("MD X: [%f] MD Y: [%f]", md.x, md.y);

		if(md.x > target.x){
			md.x -= 1;
		}else if(md.x < target.x){
			md.x += 1;
		}
		
		if(md.y > target.y){
			md.y -= 1;
		}else if(md.y < target.y){
			md.y += 1;
		}
	}

	//inicio da limpeza
	// Vector2D 			robotPose;
	// robotPose.x 		= pose.x * UNIT_FIX;
	// robotPose.y 		= pose.y * UNIT_FIX;

	// Vector2D vecRay 	= vecLaser - robotPose;
	// Vector2D uvecRay	= ~vecRay;

	// while(!uvecRay < !vecRay){

	// 	_grid->Set(
	// 		round(robotPose.x + uvecRay.x),
	// 		round(robotPose.y + uvecRay.y), 
	// 		-1);

	// 	uvecRay = uvecRay + ~vecRay;
	// }

	//set location with target increment
	_grid->Set(vecLaser.x * UNIT_FIX, vecLaser.y * UNIT_FIX, 1);
}


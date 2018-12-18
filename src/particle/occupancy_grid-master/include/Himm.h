#ifndef _HIMM_H
#define _HIMM_H

#include "geometry_msgs/Pose2D.h"

#include "OccupancyGrid.h"
#include "Vector2D.hpp"
#include "Constants.h"

class Himm{

private:
	OccupancyGrid* _grid;
public:
	Himm(OccupancyGrid* grid);
	
	void ToFile(std::string filename);
	OGCellType UpdateLocation(Pose2D pose, OGCellType dist, OGCellType theta);
	~Himm();
};

#endif /* _HIMM_H */

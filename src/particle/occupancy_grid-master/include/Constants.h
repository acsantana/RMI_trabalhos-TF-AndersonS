#ifndef _CONSTANTS_H
#define _CONSTANTS_H

//type be used within cells of occupancy grids
typedef double      OGCellType;

//@TODO verify
#define UNIT_FIX    100

/* Width and height of occupancy grids */
#define OG_WIDTH    400
#define OG_HEIGHT   500

/* Width and height of occupancy grid quadrants (1/4) */
#define OG_SEC_W    (OG_WIDTH  / 2)
#define OG_SEC_H    (OG_HEIGHT / 2)

/* Himm configuration */
#define HIMM_THRESHOLD_MAX 10
#define HIMM_THRESHOLD_MIN 0

/* Potential fields */
#define PF_THRESHOLD ((HIMM_THRESHOLD_MAX - HIMM_THRESHOLD_MIN) / 2)  /* threshould (wall if >= 50%) */
#define PF_ITERATIONS OG_HEIGHT*15 

/* Hokuyo */
#define HOKUYO_ANGLE_MIN -2.356194
#define HOKUYO_ANGLE_MAX  2.092350
#define HOKUYO_ANGLE_INC  0.006136
#define HOKUYO_RANGE_MIN  0.02
#define HOKUYO_RANGE_MAX  5.60
#define HOKUYO_LASER_SKIP 1

#endif /* _CONSTANTS_H */

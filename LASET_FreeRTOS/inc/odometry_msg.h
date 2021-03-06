#ifndef Odom_msg
#define Odom_msg


typedef struct{
	float x;
	//float y;
	//float z;
} Linear;

typedef struct{
	//float x;
	//float y;
	float z;
} Angular;

typedef struct{
	Linear linear;
	Angular angular;
} Twist;

typedef struct{
	float x;
	float y;
	//float z;
} Point;

typedef struct{
	//float x;
	//float y;
	float z;
	float w;
} Quaternion;

typedef struct{
	Point position;
	Quaternion orientation;
} Pose;


typedef struct
{	Pose pose;
	Twist twist;
} Odometry;

#endif /* Odom_msg */


Odometry init_odom_var(void);




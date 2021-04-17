#ifndef TEAM_MOVEMENT_HEADER
#define TEAM_MOVEMENT_HEADER

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <time.h>

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Odometery values
extern float posX, posY, yaw;

// Motion variables
extern float angular;
extern float linear;
extern const float SLOW_SPIN;
extern const float FAST_SPIN;
extern const float SLOW_MOVE;
extern const float FAST_MOVE;
extern const float MAX_LIN;
extern const float MAX_ROT;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);
bool monitorMotion ();
void setMotion(double dist, double linSpeed, double rot, double rotSpeed);

bool checkIfMoved(); // Monitors if the robot is moving

/** @name travel
   *  @brief Use this for controlled travel by repeatedly calling it.
   *  @param  dist Linear distance to travel (magnitude)
   *  @param  linSpeed Linear velocity (determines forwards or back)
   *  @param  rot Rotational displacement in radians (magnitude)
   *  @param  rotSpeed Rotational velocity rad/s (+ive is left)
   *  @return  If the movement is complete, true. 
   *
   *  @note It knows when you call it the first time or change course, so long as the parameters change. 
   *  (Calling travel(1,1,0,0) and then travel(1,1,0,0) again after that is completed will not register
   *  as a new "move", these need to be seperated by a unique call such as travel(0,0,0,0) to repeat)
   */
bool travel(double dist, double linSpeed, double rot, double rotSpeed);

/** @name setHeading
 * @brief Will align the robot optimally with some absolute heading.
 * @param heading The absolute heading (rad) to aim for
 * @param speed Magnitude of rotation (rad)
 * @return If the movement is complete, true.
 * */
bool setHeading(float heading, float speed);
#endif
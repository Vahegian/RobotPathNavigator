#ifndef CONSTANTS_h
#define CONSTANTS_h

const double robot_starting_pos_X = 3800;
const double robot_starting_pos_Y = 3500;
const double robot_starting_angle = 270;
const double robot_starting_pos_X_in_m = 3.8;
const double robot_starting_pos_Y_in_m = 3.5;
const double stopRange = 0.40; // robotIO will stop if obstacle is 0.35 meters away
const double moreCarefullRange = 0.60;
const double convert_millimeters_to_m = 1000.0;
const double m_to_cm = 100;
const double laser_right_angle = -90;
const double laser_left_angle = 90;
const double PI_devided_by_180 = 0.01745329;
const double robot_speed_right_wheel = 100.0;
const double robot_speed_left_wheel = 100.0;

const double robotSpeed = 100.0;
const double robotSpeed2 = 200.0; // robot travel speed

const double angle_faceing_west = 0;
const double angle_faceing_north = 89;
const double distance_from_ball = 0;

const double sonar_stop_dist = 0.25;
const double laser_stop_dist = 0.15;
const int rotation_speed = 20;
const double map_size_max = 4.3;
const double map_size_min = 0;

const double odomError = 0.2;
const double robotTurnSharpness = 30;

const double robotBallChaseSpeed = 50;
const double robotBallChaseHeading = 10;

const int white = 254;
const int obstacleAvoidRate = 20;
const int sonarSampleRate = 20;
const int laserSampleRate = 10;
const int speedSampleRate = 20;
const double distFromBall = 102; //close grip : : small num early big num late
const int sensLeft = 8;
const int sensRight = 10;

const double sonarUpperLimit = 2.0;
const double sonarLowerLimit = 0.2;
const double sonarError = 0.0125;

const double goStraigth = 1.0;
const double turnRigth = 2.0;
const double turnLeft = 3.0;



#endif
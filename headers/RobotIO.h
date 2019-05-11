#ifndef ROBOTIO_h
#define ROBOTIO_h

#include <string>
#include "Aria.h"
#include <chrono>
#include <ctime>
#include "constants.h"
#include "opencv2/opencv.hpp"
using namespace std;

class RobotIO {
public:
	void init(int argc, char **argv, void(&update)(void), void(&process)(void), void(&write)(void), void(&show)(void));
	RobotIO();
	ArLaser* getLaser();
	vector<ArSensorReading*> getSonarSensors();
	vector<double> getRobotLoc();
	ArRobot* getRobot();
	void robot_go(const double left_speed, const double right_speed);
	void robot_go2(const double speed);
	double get_wheel_speed_per_percent(const double currentSpeed, const double percent);
	void robot_turn(const string &direction, const double percent);
	void robot_turn_inplace(const string &direction, double speed);
	void robotStop();
	vector<double> getRobotVel();
	cv::VideoCapture cap;
	void setHeading(double angle);
	ArGripper * getGripper();
private:
	ArRobot robot;
	ArGripper *griper = NULL;
	// The camera (Cannon VC-C4).
	ArGlobalFunctor updateCB;
	double decrease_the_speed_left=0.0;
	double decrease_the_speed_right=0.0;
	//timerStart = chrono::system_clock::now();
	
};

#endif

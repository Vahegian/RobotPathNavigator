#ifndef BEHAVIORS_h
#define BEHAVIORS_h

#include "RobotIO.h"
#include "sensors.h"
#include "SensorCom.h"

#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <iostream>
#include "Aria.h"


using namespace std;

class Behaviors {
public:
	Behaviors();
	void init(RobotIO &robotIO, SensorCom &sensors);
	void mkWindow(const string & name, int x, int y);
	void avoidObstacles(vector<vector<double>> &laserData, double range);
	bool isObstaclesInFront(vector<vector<double>>& laserData, double range);
	bool isAtGoal(vector<double> goal, vector<double> curPos);
	bool goToLoc(vector<double> goal);
	void getBallLocation(cv::Mat frame);
	double getBallX();
	double getballY();
	//vector<double> Behaviors::getBallLocation(cv::Mat frame);
	void chaseBall(double x_coordinate);
	void stopGrip();
	void grip();
	void releaseGrip();
	void liftGrip();
	bool didBallTouchTheGripper();
	cv::Mat getFrame(cv::VideoCapture cap);
	void showFrame(const string &winName, cv::Mat &frame);
	cv::Mat& getOccImg();

private:
	RobotIO *robotIO = NULL; 
	SensorCom *sensors = NULL;
	ArGripper *gripper = NULL;
	cv::Mat *OccImg= NULL;
	vector<cv::KeyPoint> keypoints;
};

#endif
#include "Behaviors.h"

Behaviors::Behaviors() {}

void Behaviors::init(RobotIO &robotIO, SensorCom &sensors) {
	Behaviors::robotIO = &robotIO;
	Behaviors::sensors = &sensors;
	gripper = robotIO.getGripper();
	gripper->liftDown();
	//cv::namedWindow("Cam", cv::WINDOW_AUTOSIZE);// Create a window for display.
	//cv::namedWindow("Occ", cv::WINDOW_AUTOSIZE);
	OccImg = new cv::Mat(500, 500, CV_8UC(1), cv::Scalar(0, 0, 0));
}

void Behaviors::mkWindow(const string &name, int x, int y) {
	cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(name, x, y);
}

void Behaviors::avoidObstacles(vector<vector<double>> &laserData, double range)
{
	//vector<vector<double>> laserData = sensors->update_laser_sensor_readings();
	int sensNum = 0;
	for (vector<double> reading : laserData) {
		cout <<sensNum<< " Laser: " << reading.at(0)<<" : " <<reading.at(1)<< endl;
		sensNum++;
		if (reading.at(0) < range) {
			if (sensNum < sensLeft) {
				robotIO->robot_turn("left", obstacleAvoidRate);
			}
			else if (sensNum > sensRight) {
				robotIO->robot_turn("right", obstacleAvoidRate);
			}
			else {
				robotIO->robotStop();
			}
		}
	}
}

bool Behaviors::isObstaclesInFront(vector<vector<double>> &laserData, double range)
{
	//vector<vector<double>> laserData = sensors->update_laser_sensor_readings();
	//for (vector<double> reading : laserData) {
		cout << " Laser 99999: " << laserData.at(9).at(0) << endl;
		if (laserData.at(9).at(0) < range) {
			robotIO->robotStop();
			return true;
		}
		else
		{
			return false;
		}
	//}
}

bool Behaviors::isAtGoal(vector<double>goal, vector<double> curPos) {
	if ((curPos.at(0) < goal.at(1) + odomError && curPos.at(0) > goal.at(1) - odomError)
		&& (curPos.at(1) < goal.at(2) + odomError && curPos.at(1) > goal.at(2) - odomError)) {
		return true;
	}
	else
	{
		return false;
	}
}

bool Behaviors::goToLoc(vector<double> goal)
//goal.at(0)
// 1 = straight
// 2 = rigth
// 3 = left
{
	vector<double> curPos = robotIO->getRobotLoc();
	cout << curPos.at(0) << " : " << curPos.at(1)<<endl;
	if (goal.at(0) == 1) {
		if (!isAtGoal(goal, curPos)) {
			robotIO->robot_go(robotSpeed2, robotSpeed2);
			return false;
		}
		else {
			robotIO->robotStop();
			return true;
		}
	}
	else if (goal.at(0) == 2) {
		if (!isAtGoal(goal, curPos)) {
			robotIO->robot_turn("right", robotTurnSharpness);
			return false;
		}
		else {
			robotIO->robotStop();
			return true;
		}
	}
	else if (goal.at(0) == 3) {
		if (!isAtGoal(goal, curPos)) {
			robotIO->robot_turn("left", robotTurnSharpness);
			return false;
		}
		else {
			robotIO->robotStop();
			return true;
		}
	}
	else {
		robotIO->robotStop();
		return false;
	}
}

cv::Mat Behaviors::getFrame(cv::VideoCapture cap) {
	cv::Mat frame;
	cap >> frame;
	return frame;
}

void Behaviors::showFrame(const string &winName, cv::Mat &frame) {
	cv::imshow(winName, frame);
	// Stop capturing by pressing ESC.
	if (cv::waitKey(1) == 27) return;
		
}

cv::Mat & Behaviors::getOccImg()
{
	return *OccImg;
}

void Behaviors::getBallLocation(cv::Mat frame)
{
	
	//double x_coordinate;
	
	// Resize frame to 160 * 120.
	cv::resize(frame, frame, cv::Size(160, 120));
	

	//// Segment frame using orange color (B G R order).
	cv::inRange(frame, cv::Scalar(25, 60, 150), cv::Scalar(60, 120, 255), frame);
	//cv::imshow("Cam", frame);
	// Set up the detector with parameters.
	cv::SimpleBlobDetector::Params params;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByCircularity = false;
	params.minThreshold = 0;
	params.maxThreshold = 255;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByArea = true;
	params.minArea = 50;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	// Detect blobs.
	keypoints.clear();
	detector->detect(frame, keypoints);
	showFrame("Cam", frame);
	//// Return X coordinate.
	//if (keypoints.size() > 0)
	//{
	//	x_coordinate = keypoints[0].pt.x;
	//	showFrame("Cam", frame);
	//	return x_coordinate;
	//}
	//else
	//	return 0;

	
}

double Behaviors::getBallX(){
	// Return X coordinate.
	if (keypoints.size() > 0)
	{
		return keypoints[0].pt.x;
		
	}
	else
		return 0;
}
double Behaviors::getballY() {
	// Return Y coordinate.
	if (keypoints.size() > 0)
	{
		return keypoints[0].pt.y;

	}
	else
		return 0;
}

void Behaviors::chaseBall(double x_coordinate)
{
	cout << "................Chasing The ball..............." << endl;
	double xRel;
	double width = 160.0; // Image size is 160 * 120
	if (x_coordinate > 0)
	{
		// Determine where the blob's center of gravity
		// is relative to the center of the camera.
		xRel = (x_coordinate - width / 2.0) / width - 0.15;

		// Set the heading and velocity for the robot.
		if (ArMath::fabs(xRel) < .1)
		{
			robotIO->setHeading(0);
		}
		else
		{
			if (ArMath::fabs(xRel) <= 1)
				robotIO->setHeading(-xRel * robotBallChaseHeading);
				//robotIO->robot_go(-xRel * robotBallChaseHeading, 50);
			else if (-xRel > 0)
				robotIO->setHeading(robotBallChaseHeading);
				//robotIO->robot_go(50, 50);
			else
				robotIO->setHeading(-1* robotBallChaseHeading);
				//robotIO->robot_go(50, -1 * robotBallChaseHeading);
		}
		robotIO->robot_go2(robotBallChaseSpeed);
	}
	
}

void Behaviors::stopGrip(){ if (gripper->isGripMoving()) gripper->gripStop(); }
void Behaviors::grip() {
	//if (gripper->getGripState() == 1) {
	stopGrip();
	gripper->gripClose();
	//}
}

void Behaviors::releaseGrip() {
	//if (gripper->getGripState() == 2) {
	stopGrip();
	gripper->gripOpen();
	//}
}

void Behaviors::liftGrip() {
	stopGrip();
	gripper->liftUp();
	;
}

bool Behaviors::didBallTouchTheGripper() {
	if (gripper->getBreakBeamState() == 3) return true;
	else return false;
}
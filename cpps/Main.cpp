#include <iostream>
#include "SensorCom.h"
#include "FileIO.h"
#include "MapGenerator.h"
#include "Behaviors.h"

//#include "RobotIO.h"
using namespace std;
ArRobot* robot;
RobotIO robotIO;
SensorCom sensors;
FileIO fileIO;
MapGenerator mGen;
Behaviors behaviors;
cv::VideoCapture &cap = robotIO.cap;
cv::Mat frame;
cv::Mat occImg;
int nextGoal = 0;
bool initialised = false, updated = false, isBallFound = false;

vector<vector<double>> sonarPos;
vector<double> laserPos, sonarReadings, robotPos, robotSpeedReading;
vector<vector<double>> laserReadings, laserMapCoords;

vector<vector<double>> defaultGoal{ { goStraigth,robot_starting_pos_X_in_m, 1.5 }, { turnRigth, 3.0, 0.92 }, { goStraigth, 2.0, 0.92 },
									{turnRigth, 1.7, 2.0}};

void initAllObjects()
{
	fileIO.prepare_the_files();
	sensors.init(robotIO);
	behaviors.init(robotIO, sensors);
	//behaviors.releaseGrip();
	//behaviors.mkWindow("Raw", 800, 0); 
	behaviors.mkWindow("Cam", 0,600); behaviors.mkWindow("Occ", 0, 0);
	sonarPos = sensors.get_sonar_coords_on_robot();
	laserPos = sensors.get_laser_location();
	
}

void update(void) {
	if (!initialised) { initAllObjects(); initialised = true; }
	else {
		sonarReadings = sensors.update_sonar_sensor_readings();
		robotPos = robotIO.getRobotLoc();
		laserReadings = sensors.update_laser_sensor_readings();
		robotSpeedReading = robotIO.getRobotVel();
		frame = behaviors.getFrame(cap);
		behaviors.getBallLocation(frame);
		updated = true;
	}
}
bool rg = true;
void process(void)
{
	if (updated) {
		
		double x_coord = behaviors.getBallX();
		double y_coord = behaviors.getballY();
		cout << "X ::::::::::::::::::::::: " << x_coord<< "    Y "<< y_coord << endl;
		bool isObstacle = behaviors.isObstaclesInFront(laserReadings, moreCarefullRange);
		if (x_coord > 0) {
			if (!isObstacle) behaviors.chaseBall(x_coord);
			isBallFound = true;
			if (rg) {
				behaviors.releaseGrip();
				rg = false;
			}
		}
		else if (x_coord == 0 && isBallFound) {
			cout << "moving towards the goal : "<< robotPos.at(2) << endl;
			if (robotPos.at(2) < 90) robotIO.robot_turn("left", 10);
			else if (!isObstacle) robotIO.robot_go(robotSpeed, robotSpeed);
		}
		else
		{
			cout << "............Navigating.........." << endl;
			if (behaviors.goToLoc(defaultGoal.at(nextGoal))) {
				if (defaultGoal.size() - 1 != nextGoal) nextGoal++;
			}
			 behaviors.avoidObstacles(laserReadings, stopRange);
		}
		
		if ((behaviors.didBallTouchTheGripper())
															&& isBallFound) {
			behaviors.grip();
			behaviors.liftGrip();
		}
		
	}
}

void writeOutput(void)
{
	if (updated) {

		if (!robotPos.empty()) {
			fileIO.write_position(robotPos);

			vector<vector<double>> sonarMapCoords = mGen.get_SonarMap(sonarPos, sonarReadings, robotPos);
			if (!sonarMapCoords.empty()) fileIO.write_sonar_map(sonarMapCoords);

			laserMapCoords = mGen.get_LaserMap(laserPos, laserReadings, robotPos);
			if (!laserMapCoords.empty()) fileIO.write_laser_map(laserMapCoords);
		}

		if (!robotSpeedReading.empty()) fileIO.write_speed(robotSpeedReading);
	}
}

void showResults(void)
{
	if (updated) {
		//behaviors.showFrame("Raw", frame);
		occImg = behaviors.getOccImg();
		for (vector<double> coords : laserMapCoords) {
			//cout << coords.at(0) << " : " << coords.at(1) << endl;
			occImg.at<uchar>((int)(coords.at(0)*m_to_cm),(int)(coords.at(1)*m_to_cm)) = white;
		}
		behaviors.showFrame("Occ", occImg);
	}
	
	//if(!robotPos.empty()) cout << robotPos.at(1) << endl;
	//void showLaser();
}

int main(int argc, char **argv) {
	cout << "Starting Robot" << endl;
	robot = robotIO.getRobot();
	robotIO.init(argc, argv, update, process, writeOutput, showResults);
	cv::imwrite("WorldMap.png", occImg);
	fileIO.close_the_files();
	behaviors.grip();
}
#ifndef SENSORCOM_h
#define SENSORCOM_h
#include <stdlib.h>
#include <vector>
#include "Aria.h"
#include "constants.h"
#include "RobotIO.h"
using namespace std;

//vector<ArSensorReading*> vector_of_sonar_sensors;
//vector<vector<double>> sonar_sensor_coords_and_angle_on_robot;
//vector<double> laser_location;

class SensorCom {
	public:
		SensorCom();
		void init(RobotIO &robot);
		vector<vector<double>> get_sonar_coords_on_robot();
		vector<double> update_sonar_sensor_readings();
		vector<double> get_laser_location();
		vector<vector<double>> update_laser_sensor_readings();

		vector<ArSensorReading*> vector_of_sonar_sensors;
		vector<vector<double>> sonar_sensor_coords_and_angle_on_robot;
		vector<double> laser_location;
		ArLaser* laser_sensor;

	private:
		//RobotIO robotIO;
		double dist, angle;

};

#endif
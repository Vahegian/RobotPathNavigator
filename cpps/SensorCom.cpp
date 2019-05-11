#include "SensorCom.h"

SensorCom::SensorCom(){}

void SensorCom::init(RobotIO &robotio)
{	
	//robotIO = robotio;
	// create vector of sonar sensors, 
	vector_of_sonar_sensors = robotio.getSonarSensors();
	// get information about sensors locations on the robotIO coords system
	sonar_sensor_coords_and_angle_on_robot = get_sonar_coords_on_robot();
	// connect to laser sensors,
	laser_sensor = robotio.getLaser();
	laser_location = get_laser_location();
}

vector<vector<double>> SensorCom :: get_sonar_coords_on_robot()
{
	/*The sonar sensors are located on the robotIO at different location and angles.
	This method creates a vector of vectors to stores the locations and angles of all sonar sensors.
	For each sensor we have 3 values [x,y,th] where x and y are the coordinates on robotIO and 'th' is the angle
	The vector of vectors ranges from 0 to 7 where item index cooresponds to sensor nummber.
	*/
	vector<vector<double>> values;
	for (ArSensorReading *sonar : vector_of_sonar_sensors) {
		vector<double> sensor_data;

		// the gathered values are in meters because they are devided by 1000.0
		sensor_data.push_back(sonar->getSensorX() / convert_millimeters_to_m);
		sensor_data.push_back(sonar->getSensorY() / convert_millimeters_to_m);
		sensor_data.push_back(sonar->getSensorTh());
		values.push_back(sensor_data);
	}
	return values;
}

vector<double> SensorCom::get_laser_location()
{
	// Function returns a vector containing x and y coordinates of the laser on the robotIO.
	vector<double> values;
	values.push_back(laser_sensor->getSensorPositionX() / convert_millimeters_to_m);
	values.push_back(laser_sensor->getSensorPositionY() / convert_millimeters_to_m);
	return values;
}

vector<double> SensorCom::update_sonar_sensor_readings()
{
	/*
	* The function gets sonar sensor readings from sensors stored in a vector
	* creates a new vector and populates it with sensor reading, the index number of the reading
	* cooresponds to sensor number. Function then returns the vector with values.
	*/
	vector<double> values;
	for (ArSensorReading *sonar : vector_of_sonar_sensors) {
		double range = 0.0;
		for(int i=0; i<sonarSampleRate; i++)
		{
			range+= sonar->getRange() / convert_millimeters_to_m; // converts sensor values to meters
		}
		range = range / sonarSampleRate; // mean of reading measurments
		if (range > sonarLowerLimit && range < sonarUpperLimit) {
			values.push_back(range+sonarError);
		}
		else if (range > sonarUpperLimit) {
			values.push_back(sonarUpperLimit);
		}
		else {
			values.push_back(sonarLowerLimit);
		}
	}
	int n = 0;
	for (double i : values) {
		cout <<n<<" : "<< i << endl;
		n++;
	}
	return values;
}

vector<vector<double>> SensorCom::update_laser_sensor_readings()
{
	/*
	* The function gets laser sensor readings from sensor object.
	* It creates a new vector of vectors and populates it with sensor reading, where index 0 is the distance and index 1 is the angle.
	* Function then returns the vector with values.
	*
	* The laser object returnes the closest object distance and the angle at which the object is located.
	* Index of the returnd vector corresponds to the angle.
	*/


	vector<vector<double>> values;
	for (int i = laser_right_angle; i < laser_left_angle; i+=10) {
		vector<double> dist_and_angel;
		double range = 0.0;
		if (laser_sensor != 0) {
			for (int j = 0; j < laserSampleRate; j++) {
				dist = laser_sensor->currentReadingPolar(i, i + 1, &angle); // -90 to 90 gives 180 degree view on front
				range += dist / convert_millimeters_to_m;
			}
		}

		dist_and_angel.push_back(range/laserSampleRate);
		dist_and_angel.push_back(angle);
		values.push_back(dist_and_angel);
	}
	return values;
}

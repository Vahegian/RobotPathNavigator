#include "MapGenerator.h"

MapGenerator::MapGenerator(){}

double MapGenerator::convert_degrees_to_radian(double angle)
{
	// function converts degree angles to radian
	return angle*PI_devided_by_180;
}

vector<double> MapGenerator::polar_to_cartesian(double sensorX, double sensorY, double sensorTH, double distance_from_obstacle, double robotX, double robotY, double robotTH)
{
	/*
	Given sensor coordinates and angles on the robot, the distance from an obsticle and the robots position, this function will
	calculate the obstacles coordinates and will return it. The function effectively converts 'Polar' coodinats to 'Cartesian' coordinats.
	The returned vector contains x and y coordinates of an obstacle.
	*/
	double x_coord_of_obstacle_on_robot_system = sensorX + (distance_from_obstacle * cos(convert_degrees_to_radian(sensorTH)));
	double y_coord_of_obstacle_on_robot_system = sensorY + (distance_from_obstacle * sin(convert_degrees_to_radian(sensorTH)));

	double robotTH_in_radian = convert_degrees_to_radian(robotTH);

	double xr = (x_coord_of_obstacle_on_robot_system * cos(robotTH_in_radian)) - (y_coord_of_obstacle_on_robot_system * sin(robotTH_in_radian));
	double yr = (x_coord_of_obstacle_on_robot_system * sin(robotTH_in_radian)) + (y_coord_of_obstacle_on_robot_system * cos(robotTH_in_radian));

	double global_X = robotX + xr;
	double global_Y = robotY + yr;

	vector<double> coord;
	// making sure that the size of the map is not bigger than it needs to be
	if ((global_X > map_size_min && global_X < map_size_max) && (global_Y > map_size_min && global_Y <  map_size_max)) {
		coord.push_back(global_X);
		coord.push_back(global_Y);
		return coord;
	}
	else {
		coord.push_back(0);
		coord.push_back(0);
		return coord;
	}
}

vector<vector<double>> MapGenerator::get_SonarMap(vector<vector<double>> &sensorPos, vector<double> &sonar_readings, vector<double> &robot_pos)
{
	/*
	For every sonar sensor the function uses 'get_map_coords' function to calculate obstacles' coordinates.
	The Function returns vector of vectors where index of a vector coresponds to sonar sensor number
	*/
	vector<vector<double>> map_coords;
	double robotX = robot_pos.at(0);
	double robotY = robot_pos.at(1);
	double robotTH = robot_pos.at(2);

	for (int i = 0; i<sonar_readings.size(); i++)
	{
		double sonarX = sensorPos.at(i).at(0);
		double sonarY = sensorPos.at(i).at(1);
		double sonarTH = sensorPos.at(i).at(2);
		double distance_from_obstacle = sonar_readings.at(i);
		map_coords.push_back(polar_to_cartesian(sonarX, sonarY, sonarTH, distance_from_obstacle, robotX, robotY, robotTH));
	}
	return map_coords;
}

vector<vector<double>> MapGenerator::get_LaserMap(vector<double> &sensorPos, const vector<vector<double>> &laser_readings, const vector<double> &robot_pos)
{
	/*
	The finction uses 'get_map_coords' to convert 'Polar' coodinats collected from laser sensor to 'Cartesian' coordinats
	The function returns a vector containing x and y coords of an obstacle.
	*/
	vector<vector<double>> obstacle_coords;

	double laserX = sensorPos.at(0);
	double laserY = sensorPos.at(1);
	double robotX = robot_pos.at(0);
	double robotY = robot_pos.at(1);
	double robotTH = robot_pos.at(2);
	for (vector<double> laser_reading : laser_readings) {
		double distance_from_obstacle = laser_reading.at(0);
		double laserTH = laser_reading.at(1);
		obstacle_coords.push_back(polar_to_cartesian(laserX, laserY, laserTH, distance_from_obstacle, robotX, robotY, robotTH));
		//printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", laserX, laserY, laserTH, distance_from_obstacle, robotX, robotY, robotTH);
	}
	return obstacle_coords;
}
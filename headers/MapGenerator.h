#ifndef MAPGEN_h
#define MAPGEN_h
#include<vector>
#include"constants.h"
using namespace std;

class MapGenerator
{
public:
	MapGenerator();
	vector<double> polar_to_cartesian(double sensorX, double sensorY, double sensorTH, double distance_from_obstacle, double robotX, double robotY, double robotTH);
	vector<vector<double>> get_SonarMap(vector<vector<double>> &sensorPos, vector<double> &sonar_readings, vector<double> &robot_pos);
	vector<vector<double>> get_LaserMap(vector<double> &sensorPos, const vector<vector<double>> &laser_readings, const vector<double> &robot_pos);
private:
	double convert_degrees_to_radian(double angle);
};

#endif
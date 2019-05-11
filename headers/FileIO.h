#ifndef FILEIO_h
#define FILEIO_h
#include <cstdio>
#include <vector>
using namespace std;

class FileIO {
public:
	FileIO();
	void prepare_the_files();
	void write_sonar_map(const vector<vector<double>> &sonar_map);
	void write_laser_map(const vector<vector<double>> &laser_map);
	void write_position(const vector<double> &position);
	void write_speed(const vector<double> &timeAndWheelSpeed);
	void close_the_files();
private:
	FILE *fpPOS, *fpSonarMap, *fpLaserMap, *fpWheelSpeeds;

};

#endif

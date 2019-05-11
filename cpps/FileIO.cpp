#include "FileIO.h"

FileIO::FileIO(){}

void FileIO::prepare_the_files()
{
	/*The function opens 4 files in write mode, where sensor data and calculated maps will be stored.*/

	fpPOS = fopen("Pos.csv", "w");
	fprintf(fpPOS, "%s,%s\n", "x", "y"); //add heading to the file

	fpWheelSpeeds = fopen("speeds.csv", "w");
	fprintf(fpWheelSpeeds, "%s,%s,%s\n", "time", "left wheel", "right wheel"); //add heading to the file

	fpSonarMap = fopen("SonarMap.csv", "w");
	fpLaserMap = fopen("LaserMap.csv", "w");
}

void FileIO::write_position(const vector<double>& position)
{
	fprintf(fpPOS, "%.2f,%.2f\n", position.at(0), position.at(1)); // the robot position values are written on this line
}

void FileIO::write_speed(const vector<double>& timeAndWheelSpeed)
{
	fprintf(fpWheelSpeeds, "%.2f,%.2f,%.2f\n", timeAndWheelSpeed.at(0), timeAndWheelSpeed.at(1), timeAndWheelSpeed.at(2));
}

void FileIO::write_sonar_map(const vector<vector<double>>& sonar_map)
{
	for (int i = 0; i < sonar_map.size(); i++) // sonar sensor map is written to file in this 'for' loop
	{
		fprintf(fpSonarMap, "%.2f,%.2f\n", sonar_map.at(i).at(0), sonar_map.at(i).at(1));
	}
}

void FileIO::write_laser_map(const vector<vector<double>>& laser_map)
{
	for (int i = 0; i < laser_map.size(); i++) {
		fprintf(fpLaserMap, "%.2f,%.2f\n", laser_map.at(i).at(0), laser_map.at(i).at(1)); // the laser sensor map is written to file here.
	}
}

void FileIO::close_the_files()
{
	// all opened files are closed in this function
	fclose(fpPOS);
	fclose(fpWheelSpeeds);
	fclose(fpSonarMap);
	fclose(fpLaserMap);
}







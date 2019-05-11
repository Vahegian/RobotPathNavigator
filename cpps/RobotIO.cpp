#include "RobotIO.h"
RobotIO::RobotIO() {}
void RobotIO::init(int argc, char **argv, void(&update)(void), void(&process)(void), void(&write)(void), void(&show)(void)) {
	ArVCC4 vcc4(&robot);
	// Initialisation
	Aria::init();

	// Initialize the camera.
	vcc4.reset();
	// Wait for a while.
	ArUtil::sleep(100);

	// Tilt the camera down 45 degrees to make it find the ball easier.
	vcc4.tilt(-45);
	ArUtil::sleep(100);

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	// Add sonar.
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	// Connect laser.
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured laser.");
		Aria::logOptions();
		Aria::exit(1);
	}
	// connect griper
	griper = new ArGripper(&robot);

	ArPose space(3800, 3500, 270); // Initial robot's odometry.
	robot.moveTo(space); //Moves the robot's idea of its position to this position.

	// Load camera in OpenCV.
	if (!cap.open(0))
		Aria::exit(0);

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	
	ArGlobalFunctor updateFunc(&update);
	ArGlobalFunctor processFunc(&process);
	ArGlobalFunctor writeFunc(&write);
	ArGlobalFunctor showFunc(&show);
	
	robot.addUserTask("update", 50, &updateFunc);
	robot.addUserTask("process", 50, &processFunc);
	robot.addUserTask("write", 50, &writeFunc);
	robot.addUserTask("displayResults", 50, &showFunc);
	////robotIO.remUserTask("displayResults");
	robot.runAsync(true);
	robot.waitForRunExit();
	Aria::exit(0);
}

ArRobot* RobotIO::getRobot()
{
	return &robot;
}

ArGripper * RobotIO::getGripper() {
	return griper;
}

ArLaser* RobotIO::getLaser()
{
	return robot.findLaser(1);
}

vector<ArSensorReading*> RobotIO::getSonarSensors()
{
	/* The function puts all 8 sonar sensors in a vector ranging from 0 to 7 where item index cooresponds to sensor nummber
	* The items are  pointers to the sonar sensor objects
	* functoin returns vector of 'ArSensorReading' pointers.
	*/
	vector<ArSensorReading*> list_of_sensors;
	for (int i = 0; i < 8; i++) {
		list_of_sensors.push_back(robot.getSonarReading(i));
	}
	return list_of_sensors;
}

vector<double> RobotIO::getRobotLoc()
{
	/*
	* The function gets robotIO position readings from odometry.
	* It creates a new vector and populates it with position reading,
	* where index 0 is the X, index 1 is the Y position and
	* index 2 is the angle at which robotIO is headed.
	* Function then returns the vector with values.
	*/

	vector<double> values;
	values.push_back(robot.getX() / convert_millimeters_to_m);
	values.push_back(robot.getY() / convert_millimeters_to_m);
	values.push_back(robot.getTh());
	return values;
}

void RobotIO::robot_go(const double left_speed, const double right_speed) { robot.setVel2(left_speed, right_speed); }
void RobotIO::robot_go2(const double speed) { robot.setVel(speed); }

double RobotIO::get_wheel_speed_per_percent(const double currentSpeed, const double percent)
{
	return ((currentSpeed / 100.0)*percent);
}

void RobotIO::robot_turn(const string &direction, const double percent)
{
	/*
	* given the 'direction' and the 'percent' the function slows down right or left wheel of the
	* robot to make a smooth turn.
	* when 'direction' is 'right' the speed of the right wheel decreses according to the 'percent'
	* when 'direction' is 'left' the speed of the left wheel decreses according to the 'percent'
	*/

	if (!direction.compare("right"))
	{
		decrease_the_speed_right = get_wheel_speed_per_percent(robot_speed_right_wheel, percent);
		robot_go(robot_speed_left_wheel, robot_speed_right_wheel - decrease_the_speed_right);
	}
	else if (!direction.compare("left"))
	{
		decrease_the_speed_left = get_wheel_speed_per_percent(robot_speed_left_wheel, percent);
		robot_go(robot_speed_left_wheel - decrease_the_speed_left, robot_speed_right_wheel);
	}
}

void RobotIO::robot_turn_inplace(const string& direction, double speed)
{
	if (!direction.compare("right"))
	{
		robot.setVel2(speed, -1 * speed);
	}
	else if (!direction.compare("left"))
	{
		robot.setVel2(-1*speed, speed);
	}
}

void RobotIO::robotStop()
{
	robot.stop();
}

vector<double> RobotIO::getRobotVel()
{
	vector<double> speeds;
	//chrono::duration<double> time_passed = chrono::system_clock::now() - timerStart;
	speeds.push_back(robot.getOdometerTime());
	double vhelLeft = 0.0, vhelRight = 0.0;
	for (int j = 0; j < speedSampleRate; j++) {
		vhelLeft += robot.getLeftVel();
		vhelRight += robot.getRightVel();
	}
	speeds.push_back(vhelLeft/speedSampleRate);
	speeds.push_back(vhelRight /speedSampleRate);
	return speeds;
}

void RobotIO::setHeading(double angle) {
	cout << "angle: " << angle << endl;
	robot.setDeltaHeading(angle);
}




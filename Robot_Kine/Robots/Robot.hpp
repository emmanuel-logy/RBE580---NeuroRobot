//============================================================================
// Name        : Robot.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is strictly for defining an Abstract Robot Class
//				 all children of this type must implement the following
//============================================================================

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "eigen3/Eigen/Dense"
#include <string>
#include <vector>
using namespace std;

// This struct defines parameters of the robot probe
struct Probe
{
	double _cannulaToTreatment;
	double _treatmentToTip;
	double _robotToEntry;
	double _robotToTreatmentAtHome;
};

class Robot
{
public:
	//================ Constructor ================
	Robot(){};
	virtual ~Robot(){};

	//================ Parameters =================
	// An object of type robot must have the following parameters
public:
	string _name;				 // Name of the given robot
//	string _mode;				 // Defines the current robot mode
//	string _socketIGTConnection; // Defines the status of the IGT Connection, the string is "Connected" when an IGT socket is connected
//	string _imagerStatus;		 // Defines the current status the Imager (ie: MRI, Camera's, Ultrasound, etc), the status can report the stage of the procedure (ie "Valid Target", etc)

	Probe _probe; // Defines the specific probe structure on the robot

	// Flags
//	int FlagCalibration{0};						 // Flag for when a registration is set
//	int FlagTarget{0};							 // Flag for when a target point is set
//	int FlagEntry{0};							 // Flag for when an entry point is set
	Eigen::Matrix4d _registration;				 // Transformation between the imager and the robot's reference frame
	Eigen::Matrix4d _targetPointFullPoseScanner; // Target pose as sent from the navigation
	Eigen::Matrix4d _entryPointFullPoseScanner;	 // Entry pose as sent from the navigation

//	int _punctureDetectionsLeft;	// Specifcies ....
//	vector<double> _membraneDepths; // Specifies estimates for membrane depths to detect
//	vector<string> _robotModes;		// Specifies robot specific methods

	Eigen::Matrix4d _currentPose; // Transformation that represents current location of the treatment zone
	Eigen::Matrix4d _targetPose;  // Represents reachable target by the robot's treatment zone in RAS
	Eigen::Matrix4d _imagerTip;	  // Transformation between the imager and the robot's treatment zone with respect to imager

	Eigen::Vector3d _entryPoint;  // Entry point for the desired pass-through location of the treatment zone
	Eigen::Vector3d _targetPoint; // Target point for the final location of the treatment zone

//	vector<vector<double>> _trajectory; // Defines a set of time stamped joint positions that the robot should follow
//	unsigned int _trajectoryIndex = 0;	// To preserve asynchronous behavior, global trajectory index gives the current setpoint the robot should follow

	//================ Public Methods =================
	// An object of type robot must have the following parameters
	// Main Method responsible for ....
//	virtual void Update() = 0;

	// Robot specific Methods for various robot specific functionalities
//	virtual void HomeRobot() = 0;
//	virtual void StopRobot() = 0;
//	virtual void ZeroRobot() = 0;
//	virtual void UpdateRobot(int usPeriod) = 0;

	// Robot specific helper methods needed by front end gui
	// Helper method to validate a motor setponint by front end gui
//	virtual vector<int> Axis_Setpoint_Validator() = 0;
	// Helper method to validate the output of the inverse kinematic
	virtual vector<int> Axis_Setpoint_Validator(vector<int> &) = 0;
	virtual void RunInverseKinematics() = 0;
//	virtual int IsFootPedalPressed() = 0;

	// Robot specific Motor related helper Methods
//	virtual Motor *GetMotor(int cardID) = 0;
//	virtual vector<Motor *> ListMotors() = 0;
//	virtual bool CheckForStalls() = 0;
};

#endif /* ROBOT_HPP_ */

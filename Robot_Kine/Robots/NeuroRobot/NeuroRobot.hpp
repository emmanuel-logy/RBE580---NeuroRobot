//============================================================================.
// Name        : NeuroRobot.hpp.
// Author      : Produced in the WPI AIM Lab.
// Description : This file defines the methods and parameters of the class Neurosurgery Robot.
//============================================================================

#ifndef NEUROROBOT_HPP_
#define NEUROROBOT_HPP_

using namespace std;
#include <string>

#include "../../Robots/Robot.hpp"
#include "../../Kinematics/NeuroKinematics/NeuroKinematics.hpp"
#include "eigen3/Eigen/Dense"

class NeuroRobot : public Robot
{
public:
	//================ Constructor ================
	NeuroRobot();
	virtual ~NeuroRobot(){};

	//================ Parameters =================
	// Parameters are initialized in constructor
	// See the Robot.hpp file for declaration of robot specific variables such as : _name, _registration, _probe, etc
	// See the NeuroRobot.cpp file for implementation of these robot specific variables for the NeuroRobot

//	// Packet Information for SPI Communications
//	Packets *_packets;
//
//	// The FPGA Utility object was included to allow for control over LEDs
//	FPGA_Utilities *_fpga_util;
//
//	// Timing related variables
//	Timer _timer;
//	int _loopRate;
//
//	// Frequency Sweep specific variable
//	int _frequency;

	//=================== Neuro Robot Kinematics ==================
	// This class contains the forward and inverse kinematics for this robot
	NeuroKinematics _neuroKinematics;

	//=================== NeuroRobot Motors =======================
	// NeuroRobot Motors are defined below:
	struct MotorConfig
	{
		double _ticksPerUnit;
		double _minTicks;
		double _maxTicks;
		int _setpoint = 0;
	};


	MotorConfig _yawRotation;
	MotorConfig _pitchRotation;
	MotorConfig _probeRotation;
	MotorConfig _probeInsertion;
	MotorConfig _lateralTranslation;
	MotorConfig _axialFeetTranslation;
	MotorConfig _axialHeadTranslation;


	// -- Insertion Motors
//	Motor_Config ProbeInsertionConfig;
//	Motor _probeInsertion;
//
//	// -- Orientation Motors
//	Motor_Config YawRotationConfig, PitchRotationConfig, ProbeRotationConfig;
//	Motor _yawRotation, _pitchRotation, _probeRotation;
//
//	// -- Axial Motors
//	Motor_Config AxialHeadTranslationConfig, AxialFeetTranslationConfig, LateralTranslationConfig, PlasticMotorConfig;
//	Motor _axialHeadTranslation, _axialFeetTranslation, _lateralTranslation;

	//================ Public Methods ==============
	// Methods that must be implemented from the abstract Robot Class
//	void Update(); // This is the central robot functionality method, and this is called from our main loop in SurgicalRobot.cpp
//	void HomeRobot();
//	void StopRobot();
//	void ZeroRobot();
//	void DrapeRobot();
//	void AnimalTrialRotateRobot();
//	void UpdateRobot(int usPeriod);
	void RunFK();
	void RunInverseKinematics();
//	std::vector<int> Axis_Setpoint_Validator();
	std::vector<int> Axis_Setpoint_Validator(std::vector<int> &);
//	int IsFootPedalPressed();
//	Motor *GetMotor(int cardID);
//	vector<Motor *> ListMotors();
//	bool CheckForStalls();

	// Other Methods specific only to the NeuroRobot
//	void FollowTrajectory();
//	void FrequencySweep(int cardID);
//	bool DetectMotorStateChange();
//	void CoordinatedControlCalculateIncrements(double N);
//	bool AllMotorSubSetpointsReached();
//	void CoordinatedControlUpdateSetpoints();
	void CreateWorkspace();
};

#endif /* NEUROROBOT_HPP_ */

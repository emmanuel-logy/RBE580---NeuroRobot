//============================================================================
// Name        : NeuroRobot.cpp
// Author      : Produced in the WPI AIM Lab
// Description : This file defines the methods and parameters of the Neurosurgery Robot
//============================================================================

#include <iostream>
#include <utility>
#include <fstream>
#include "NeuroRobot.hpp"
using namespace std;


int LOG_LEVEL_WARNING = 0;
int LOG_LEVEL_INFO = 1;
class Logger
{
public:
	void Log(const string& msg, int logLevel, bool whatever)
	{
		cout << msg << "\n\n";
	}

	static Logger& GetInstance()
	{
		static Logger instance; // Guaranteed to be destroyed.
							  // Instantiated on first use.
		return instance;
	}

private:

};




// Neuro Robot Constructor
// Requires Pointers to a Packets class (used for global access to SPI data)
// And an FPGA_Utility class (used for global access to FPGA specific functions ie LEDs)
NeuroRobot::NeuroRobot()
{

	// Name of the robot used for front-end GUIs and menus
	_name = "Neuro Robot";

//	// Packet Information for SPI Communications
//	_packets = packets;
//
//	// The FPGA Utility object was included to allow for control over LEDs
//	_fpga_util = fpga_util;
//
//	// Robot Mode used to specify specific robot states for asynchronous functionalities
//	_mode = "";
//	_robotModes = {"Manual", "Home", "Draping", "Rotate Probe Animal Trial", "Frequency Sweep", "Trajectory", "Coordinated Move"};
//
//	// Timing related variables
//	_timer = Timer();
//	_loopRate = loopRate;
//
//	// These variables are received via openIGTLink
//	string _imagerStatus = "";
//	string _socketIGTConnection = "";
//	_membraneDepths = {};
//	_trajectory = {};
//	_trajectoryIndex = 0;
//
//	// Frequency Sweep specific variables
//	_frequency = 0x48000000;

	//=================== Neuro Robot Probes ======================
	// Defines the specific probe structure on the robot
	//_probe = { canulaToTreatment, treatmentToTip, robotToEntry, robotToTreatmentAtHome;}
	_probe = {0, 25, 0, 40};

	//=================== Neuro Robot Kinematics ==================
	// This class stores functions for forward and inverse kinematics
	_neuroKinematics = NeuroKinematics(&_probe);
	_targetPointFullPoseScanner.setIdentity();

	//=================== Neuro Robot Motors ==================
	// Motor Configurations -- For more information see Motor.hpp
	// TODO: The probe rotation unit constant is wrong.....0x43f26700
	// Motor = { card_type, motor_type, name, encoderResolution, unit, ticksPerUnit, motorDirection, directionCorrection, limitType, maxVelocitys, minVelocity, minTicks, maxTicks, _homeOffsetInTicks, Kp, Ki, Kd, deadband };
//	YawRotationConfig = {card_type::externaldriver_pwm, motor_type::shinsei, "Yaw Rotation", 5000, "degree", 795.8, 1, 1, limit_type::upper, 0x0000efff, 0x00000fff, -1222, 8, 0, 100, 0, 0, 8}; // Original Max Velocity: 0x0001ffff
//	ProbeRotationConfig = {card_type::externaldriver_pwm, motor_type::shinsei, "Probe Rotation", 5000, "degree", 1285.5, 1, 1, limit_type::unspecified, 0x0000ffff, 0x0, -8073, 8073, 0, 100, 0, 0, 20};
//	PitchRotationConfig = {card_type::externaldriver_non_pwm, motor_type::shinsei, "Pitch Rotation", 5000, "degree", 795.8, 0, 1, limit_type::upper, 0x0001ffff, 0x0, -370, 516, 0, 100, 0, 0, 5};
//	ProbeInsertionConfig = {card_type::externaldriver_pwm, motor_type::shinsei, "Probe Insertion", 5000, "mm", 3333.34, 0, 0, limit_type::lower, 0x0001ffff, 0x0, 0, 133334, 0, 1, 1, 0, 3};
//	LateralTranslationConfig = {card_type::externaldriver_non_pwm, motor_type::shinsei, "Lateral Translation", 5000, "mm", 47.24, 1, 1, limit_type::upper, 0x0001ffff, 0x0, -2337, 0, 0, 100, 0, 0, 5};
//	AxialFeetTranslationConfig = {card_type::externaldriver_non_pwm, motor_type::shinsei, "Axial Feet Translation", 5000, "mm", 78.74, 0, 1, limit_type::unspecified, 0x0001ffff, 0x0, -5512, 5906, 0, 100, 0, 0, 3};
//	AxialHeadTranslationConfig = {card_type::externaldriver_non_pwm, motor_type::shinsei, "Axial Head Translation", 5000, "mm", 78.74, 1, 1, limit_type::upper, 0x0001ffff, 0x0, -11418, 0, 0, 100, 0, 0, 3};


	// These are the motor definitions.... {ticksPerUnit, minTicks, maxTicks};
	_yawRotation 			= {795.8,	-1222,	8		};		// -1.57 <-->  0.01 rad
	_pitchRotation 			= {795.8,	-370,	516		};		// -0.46 <--> -0.64 rad
	_probeRotation 			= {1285.5,	-8073,	8073	};		// -6.28 <-->  6.28 rad
	_probeInsertion 		= {3333.34,	0, 		133334	};		//  0	 <-->  40.00 mm
	_lateralTranslation 	= {5000,	-2337, 	0		};		// -0.46 <--> 0 mm
	_axialFeetTranslation 	= {5000,	-5512, 	5906	};		// -1.1  <--> -1.18 mm
	_axialHeadTranslation 	= {5000,	-11418, 0		};		// -2.28 <--> 0 mm


//	_probeRotation = Motor(packets, 1, ProbeRotationConfig);
//	_pitchRotation = Motor(packets, 2, PitchRotationConfig);
//	_probeInsertion = Motor(packets, 3, ProbeInsertionConfig);
//	_lateralTranslation = Motor(packets, 4, LateralTranslationConfig);
//	_axialFeetTranslation = Motor(packets, 6, AxialFeetTranslationConfig);
//	_axialHeadTranslation = Motor(packets, 7, AxialHeadTranslationConfig);

	//=================== Neuro Robot Poses ======================
	// Transformation from scanner to robot zFrame
	_registration = Eigen::Matrix4d::Identity();
//	_registration = (Eigen::Matrix4d() << 1, 0, 0, 195,
//					 0, 1, 0, -31.8,
//					 0, 0, 1, 74.0,
//					 0, 0, 0, 1)
//						.finished();

	// Current and Target pose are defaulted to identity
	// These transformations are with respect to the Imager Coordinate Frame
	_currentPose = Eigen::Matrix4d::Identity();
	_targetPose = Eigen::Matrix4d::Identity();

	// Default values for the entry and target points
//	Eigen::Vector3d e1(18.46, 229, 72.91);
//	Eigen::Vector3d t1(18.46, 219, 72.91);

	Eigen::Vector3d p(-35, 185, 30);
	Eigen::Vector3d e1(-36.5, 158.4, 84.2);
//	e1 = e1 + p;
	Eigen::Vector3d t1(-36.5, 158.4, 88.2);
//	t1 = t1 + p;
	_entryPoint = e1;
	_targetPoint = t1;
}

/*
// This is the central NeuroRobot method -- everything happens in this method
// This method should never be blocking, it must be asynchronous
void NeuroRobot::Update()
{
	// Update sensor and encoder readings via the SPI packets
	UpdateRobot(_loopRate);

	// Update these motor setpoints without the footpedal
	if (_mode == "Draping")
	{
		DrapeRobot();
	}
	else if (_mode == "Rotate Probe Animal Trial")
	{
		AnimalTrialRotateRobot();
	}

	// If the Footpedal is pressed
	if (_fpga_util->IsFootPedalPressed())
	{
		// ============ ROBOT MODES UNRELATED TO BASE ROBOT MOVE FUNCTION ============
		if (_mode == "Manual")
		{
			//_timer.tic(); // SetPoints are given by the User via the Sliders
		}
		else if (_mode == "Home")
		{
			HomeRobot(); // SetPoints are given by the Home Method
		}
		else if (_mode == "Trajectory")
		{
			FollowTrajectory(); // SetPoints are given by the Trajectory
		}
		else if (_mode == "Frequency Sweep")
		{
			FrequencySweep(1); // Sweep Frequencies for a specific motor
		}

		// ================== COORDINATED MOVE MODE ==================
		//		if(_mode == "Coordinated Move"){
		//			// Check if any new motors have become enabled
		//			// Check if any motor desired set points have changed in ui
		//			if(DetectMotorStateChange()){
		//				// Recalculate motor increments
		//				CoordinatedControlCalculateIncrements(500);
		//			}
		//
		//			// Check if all motors have reached the motor increment set point
		//			if(AllMotorSubSetpointsReached()){
		//				// Calculate next motor set point
		//				CoordinatedControlUpdateSetpoints();
		//			}
		//
		//			CoordinatedMove();
		//		} else{

		// Move motors to their setpoints
		_yawRotation.MoveMotor();
		_probeRotation.MoveMotor();
		_pitchRotation.MoveMotor();
		_probeInsertion.MoveMotor();
		_lateralTranslation.MoveMotor();
		_axialHeadTranslation.MoveMotor();
		_axialFeetTranslation.MoveMotor();
	}

	// If the footpedal is not pressed
	else
	{
		// Update Motor Stall Detect
		_yawRotation.UpdateMotorStallTime();
		_probeRotation.UpdateMotorStallTime();
		_pitchRotation.UpdateMotorStallTime();
		_probeInsertion.UpdateMotorStallTime();
		_lateralTranslation.UpdateMotorStallTime();
		_axialHeadTranslation.UpdateMotorStallTime();
		_axialFeetTranslation.UpdateMotorStallTime();

		_probeInsertion._encoder._timer.tic();
		_yawRotation._encoder._timer.tic();
		// Stop all the motors of the robot
		StopRobot();
	}

	CheckForStalls();

	// As the current position of the robot changes, update _currentPose transform
	Eigen::Matrix4d fk = _neuroKinematics.ForwardKinematics(
											 _axialHeadTranslation.GetEncoderPositionUnit(), _axialFeetTranslation.GetEncoderPositionUnit(),
											 _lateralTranslation.GetEncoderPositionUnit(), _probeInsertion.GetEncoderPositionUnit(),
											 _probeRotation.GetEncoderPositionUnit(), _pitchRotation.GetEncoderPositionUnit(), _yawRotation.GetEncoderPositionUnit())
							 .zFrameToTreatment;

	_currentPose = _registration * fk;
}

bool NeuroRobot::CheckForStalls()
{
	vector<Motor *> motors = ListMotors();
	for (Motor *motor : motors)
	{
		if (motor->_log_stall_failure_flag > 0)
		{
			return true;
		}
	}

	return false;
}

// Method for homing the neurorobot
void NeuroRobot::HomeRobot()
{
	// TODO: Populate this function completely
}

// Method that stops all motors of the neurorobot by disabling amp enable
void NeuroRobot::StopRobot()
{
	_yawRotation.StopMotor();
	_probeRotation.StopMotor();
	_pitchRotation.StopMotor();
	_probeInsertion.StopMotor();
	_lateralTranslation.StopMotor();
	_axialHeadTranslation.StopMotor();
	_axialFeetTranslation.StopMotor();
}

// Method that zeroes the encoders by setting the encoder reference to the current encoder reading
// See Encoder.cpp for more information on how the encoder value is calculated
void NeuroRobot::ZeroRobot()
{
	_yawRotation.SetEncoderReference();
	_probeRotation.SetEncoderReference();
	_pitchRotation.SetEncoderReference();
	_probeInsertion.SetEncoderReference();
	_lateralTranslation.SetEncoderReference();
	_axialHeadTranslation.SetEncoderReference();
	_axialFeetTranslation.SetEncoderReference();
}

// Method that is used to put the robot into a suitable configuration for draping
void NeuroRobot::DrapeRobot()
{
	_yawRotation._setpoint = -596.85;		   // -0.75rad
	_pitchRotation._setpoint = 358.11;		   // 0.45rad
	_axialHeadTranslation._setpoint = -3543.3; // -45mm
}

// Method that is used to set the robot to the experimental rotation for animal trials
void NeuroRobot::AnimalTrialRotateRobot()
{
	_probeRotation._setpoint = 2018.23; // +1.57rad (+90deg)
}

// This method updates the readings from the encoder and sensors via SPI packets
void NeuroRobot::UpdateRobot(int usPeriod)
{
	// Update Encoder Readings
	_yawRotation.UpdateEncoder(usPeriod);
	_probeRotation.UpdateEncoder(usPeriod);
	_pitchRotation.UpdateEncoder(usPeriod);
	_probeInsertion.UpdateEncoder(usPeriod);
	_lateralTranslation.UpdateEncoder(usPeriod);
	_axialHeadTranslation.UpdateEncoder(usPeriod);
	_axialFeetTranslation.UpdateEncoder(usPeriod);

	// TODO: Add NeuroRobot Sensors as needed
}
*/

Neuro_FK_outputs FK_output_mid;
Neuro_FK_outputs FK_output_min;
Neuro_FK_outputs FK_output_max;


void NeuroRobot::RunFK()
{

	pair<double, double> AxialHeadTranslation {_axialHeadTranslation._minTicks / _axialHeadTranslation._ticksPerUnit,
											   _axialHeadTranslation._maxTicks / _axialHeadTranslation._ticksPerUnit};

	pair<double, double> AxialFeetTranslation {_axialFeetTranslation._minTicks / _axialFeetTranslation._ticksPerUnit,
											   _axialFeetTranslation._maxTicks / _axialFeetTranslation._ticksPerUnit};

	pair<double, double> LateralTranslation {_lateralTranslation._minTicks / _lateralTranslation._ticksPerUnit,
											 _lateralTranslation._maxTicks / _lateralTranslation._ticksPerUnit};

	pair<double, double> ProbeInsertion {_probeInsertion._minTicks / _probeInsertion._ticksPerUnit,
										 _probeInsertion._maxTicks / _probeInsertion._ticksPerUnit};

	pair<double, double> ProbeRotation {_probeRotation._minTicks / _probeRotation._ticksPerUnit,
									 	_probeRotation._maxTicks / _probeRotation._ticksPerUnit};

	pair<double, double> YawRotation {_yawRotation._minTicks / _yawRotation._ticksPerUnit,
									  _yawRotation._maxTicks / _yawRotation._ticksPerUnit};

	pair<double, double> PitchRotation {_pitchRotation._minTicks / _pitchRotation._ticksPerUnit,
										_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit};


	Neuro_FK_outputs FK_output;

	// [1] Get min reachable positions
	cout << "----------------FK - min reachable positions ----------------------" << endl;
	FK_output_min = _neuroKinematics.ForwardKinematics(AxialHeadTranslation.first, AxialFeetTranslation.first,
												   LateralTranslation.first, ProbeInsertion.first,
												   ProbeRotation.first, PitchRotation.first, YawRotation.first);
	double x = FK_output_min.zFrameToTreatment(0,3);
	double y = FK_output_min.zFrameToTreatment(1,3);
	double z = FK_output_min.zFrameToTreatment(2,3);
//	cout << x << ", " << y << ", " << z << endl;
	cout << FK_output_min.zFrameToTreatment << endl;

	cout << "FK _axialHeadTranslation input: " << -1.57*_axialHeadTranslation._ticksPerUnit << endl;
	cout << "FK _axialFeetTranslation input: " << -0.9*_axialFeetTranslation._ticksPerUnit << endl;
	cout << "FK _lateralTranslation input: " << -0.25*_lateralTranslation._ticksPerUnit << endl;
	cout << "FK _probeInsertion input: " << 25*_probeInsertion._ticksPerUnit << endl;
	cout << "FK _probeRotation input: " << 1.57*_probeRotation._ticksPerUnit << endl;
	cout << "FK _pitchRotation input: " << 0.5*_pitchRotation._ticksPerUnit << endl;
	cout << "FK _yawRotation input: " << -1.0*_yawRotation._ticksPerUnit << endl;


	// [2] Get max reachable positions
	cout << "----------------FK - max reachable positions ----------------------" << endl;
	FK_output_max = _neuroKinematics.ForwardKinematics(AxialHeadTranslation.second, AxialFeetTranslation.second,
												   LateralTranslation.second, ProbeInsertion.second,
												   ProbeRotation.second, PitchRotation.first, YawRotation.second);
	x = FK_output_max.zFrameToTreatment(0,3);
	y = FK_output_max.zFrameToTreatment(1,3);
	z = FK_output_max.zFrameToTreatment(2,3);
//	cout << x << ", " << y << ", " << z << endl;
	cout << FK_output_max.zFrameToTreatment << endl;


	// [3] Get mid reachable positions
//	_yawRotation 			= {795.8,	-1222,	8		};		// -1.57 <-->  0.01 rad
//	_pitchRotation 			= {795.8,	-370,	516		};		// -0.46 <-->  0.64 rad
//	_probeRotation 			= {1285.5,	-8073,	8073	};		// -6.28 <-->  6.28 rad
//	_probeInsertion 		= {3333.34,	0, 		133334	};		//  0	 <-->  40.00 mm
//	_lateralTranslation 	= {5000,	-2337, 	0		};		// -0.46 <-->  0 mm
//	_axialFeetTranslation 	= {5000,	-5512, 	5906	};		// -1.1  <-->  1.18 mm
//	_axialHeadTranslation 	= {5000,	-11418, 0		};		// -2.28 <-->  0 mm
	cout << "----------------FK - mid reachable positions ----------------------" << endl;
	FK_output_mid = _neuroKinematics.ForwardKinematics(-1.57, -0.9,
													   -0.25, 25,
													   1.57, 0.5, -1.0);

//	cout << "FK _axialHeadTranslation input: " << -1.57*_axialHeadTranslation._ticksPerUnit << endl;
//	cout << "FK _axialFeetTranslation input: " << -0.9*_axialFeetTranslation._ticksPerUnit << endl;
//	cout << "FK _lateralTranslation input: " << -0.25*_lateralTranslation._ticksPerUnit << endl;
//	cout << "FK _probeInsertion input: " << 25*_probeInsertion._ticksPerUnit << endl;
//	cout << "FK _probeRotation input: " << 1.57*_probeRotation._ticksPerUnit << endl;
//	cout << "FK _pitchRotation input: " << 0.5*_pitchRotation._ticksPerUnit << endl;
//	cout << "FK _yawRotation input: " << -1.0*_yawRotation._ticksPerUnit << endl;

	x = FK_output_mid.zFrameToTreatment(0,3);
	y = FK_output_mid.zFrameToTreatment(1,3);
	z = FK_output_mid.zFrameToTreatment(2,3);
//	cout << x << ", " << y << ", " << z << endl;
	cout << FK_output_mid.zFrameToTreatment << endl;
}


void NeuroRobot::CreateWorkspace()
{
	Neuro_FK_outputs FK_output;

	pair<double, double> AxialHeadTranslation {_axialHeadTranslation._minTicks / _axialHeadTranslation._ticksPerUnit,
											   _axialHeadTranslation._maxTicks / _axialHeadTranslation._ticksPerUnit};

	pair<double, double> AxialFeetTranslation {_axialFeetTranslation._minTicks / _axialFeetTranslation._ticksPerUnit,
											   _axialFeetTranslation._maxTicks / _axialFeetTranslation._ticksPerUnit};

	pair<double, double> LateralTranslation {_lateralTranslation._minTicks / _lateralTranslation._ticksPerUnit,
											 _lateralTranslation._maxTicks / _lateralTranslation._ticksPerUnit};

	pair<double, double> ProbeInsertion {_probeInsertion._minTicks / _probeInsertion._ticksPerUnit,
										 _probeInsertion._maxTicks / _probeInsertion._ticksPerUnit};

	pair<double, double> ProbeRotation {_probeRotation._minTicks / _probeRotation._ticksPerUnit,
									 	_probeRotation._maxTicks / _probeRotation._ticksPerUnit};

	pair<double, double> YawRotation {_yawRotation._minTicks / _yawRotation._ticksPerUnit,
									  _yawRotation._maxTicks / _yawRotation._ticksPerUnit};

	pair<double, double> PitchRotation {_pitchRotation._minTicks / _pitchRotation._ticksPerUnit,
										_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit};


	string filepath = "workspace.txt";
	ofstream myout;
	myout.open(filepath, ios::out | ios::trunc);

	//	_yawRotation 			= {795.8,	-1222,	8		};		// -1.57 <-->  0.01 rad
	//	_pitchRotation 			= {795.8,	-370,	516		};		// -0.46 <-->  0.64 rad
	//	_probeRotation 			= {1285.5,	-8073,	8073	};		// -6.28 <-->  6.28 rad
	//	_probeInsertion 		= {3333.34,	0, 		133334	};		//  0	 <-->  40.00 mm
	//	_lateralTranslation 	= {5000,	-2337, 	0		};		// -0.46 <-->  0 mm
	//	_axialFeetTranslation 	= {5000,	-5512, 	5906	};		// -1.10 <-->  1.18 mm
	//	_axialHeadTranslation 	= {5000,	-11418, 0		};		// -2.28 <-->  0 mm

	// Insert content
	double x=0.0; 	double y=0.0;	double z=0.0;
	cout << "AxialHead_min" << AxialHeadTranslation.first << endl;
	cout << "AxialHead_max" << AxialHeadTranslation.second<< endl;

	double k=0.1;	// reduce to 0.01 for more precise workspace
	for (double a=AxialHeadTranslation.first; a<=AxialHeadTranslation.second; a=a+k)
	{
		cout << a << endl;
		for (double b=AxialFeetTranslation.first; b<=AxialFeetTranslation.second; b=b+k)
		{
			for (double c=LateralTranslation.first; c<=LateralTranslation.second; c=c+k)
			{
				for (double d=ProbeInsertion.first; d<=ProbeInsertion.second; d=d+5)
				{
					// ProbeRotation doesn't add any new points in the point cloud
					for (double f=YawRotation.first; f<=YawRotation.second; f=f+k)
					{
						for (double g=PitchRotation.first; g<=PitchRotation.second; g=g+k)
						{
							FK_output = _neuroKinematics.ForwardKinematics(a, b,
																		   c, d,
																		   ProbeRotation.first+0.5, f, g);

							x = FK_output.zFrameToTreatment(0,3);
							y = FK_output.zFrameToTreatment(1,3);
							z = FK_output.zFrameToTreatment(2,3);

							myout << x << " " << y << " " << z << "\n";
						}
					}
				}
			}
		}
	}

	// Closing file
	myout.close();

}


void NeuroRobot::RunInverseKinematics()
{
	// Entry and Target Points are received in scanner coordinates
	// Multiply them by the registration matrix to obtain them in zFrame coordinates
	Eigen::Vector4d entryPointScannerCoordinates(_entryPoint(0), _entryPoint(1), _entryPoint(2), 1);
	Eigen::Vector4d targetPointScannerCoordinates(_targetPoint(0), _targetPoint(1), _targetPoint(2), 1);


	// Calculate zFrameToEntry
	Eigen::Vector4d zFrameToEntry = _registration.inverse() * entryPointScannerCoordinates;
	Eigen::Vector4d zFrameToTargetPt = _registration.inverse() * targetPointScannerCoordinates;

	// Calculate zFrameToTarget
//	_targetPointFullPoseScanner(0,3) = _targetPoint(0);
//	_targetPointFullPoseScanner(1,3) = _targetPoint(1);
//	_targetPointFullPoseScanner(2,3) = _targetPoint(2);
//	Eigen::Matrix3d rot;
//	rot << 0,  		0.85,  0.52,
//		   0.84,    0.28,  -0.46,
//		   -0.54,   0.44,  -0.717;
//	_targetPointFullPoseScanner.block<3,3>(0,0) = rot;
	_targetPointFullPoseScanner = FK_output_max.zFrameToTreatment;

	Eigen::Matrix4d zFrameToTarget = _registration.inverse() * _targetPointFullPoseScanner;
	// Perform Inverse Kinematics
	Neuro_IK_outputs IK_output;
	IK_output = _neuroKinematics.InverseKinematics(zFrameToEntry, zFrameToTarget);

	// Original Target Pose Calculation -- Before Reachable Target Change
	Eigen::Matrix4d _desiredTargetPose = _registration * (Eigen::Matrix4d() << IK_output.targetPose).finished();

	// Set temporary setpoints based on the output of the IK method
	std::vector<int> ikCalculatedSetPoints{};
	ikCalculatedSetPoints.push_back(IK_output.AxialHeadTranslation * _axialHeadTranslation._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.AxialFeetTranslation * _axialFeetTranslation._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.LateralTranslation * _lateralTranslation._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.ProbeInsertion * _probeInsertion._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.ProbeRotation * _probeRotation._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.PitchRotation * _pitchRotation._ticksPerUnit);
	ikCalculatedSetPoints.push_back(IK_output.YawRotation * _yawRotation._ticksPerUnit);

	// Axis validation
	vector<int> validAxisSetpoints = Axis_Setpoint_Validator(ikCalculatedSetPoints);

	// Set motor values based on validated setpoints
	_axialHeadTranslation._setpoint = validAxisSetpoints.at(0);
	_axialFeetTranslation._setpoint = validAxisSetpoints.at(1);
	_lateralTranslation._setpoint = validAxisSetpoints.at(2);
	_probeInsertion._setpoint = validAxisSetpoints.at(3);
	_probeRotation._setpoint = validAxisSetpoints.at(4);
	_pitchRotation._setpoint = validAxisSetpoints.at(5);
	_yawRotation._setpoint = validAxisSetpoints.at(6);


	cout << "\n\n\n---********------*******-------" << endl;
	for (int i=0; i<ikCalculatedSetPoints.size(); ++i)
	{
		cout << "IK output \t\t: " << ikCalculatedSetPoints.at(i) << "\n";
		cout << "Setpoint  \t\t: " << validAxisSetpoints.at(i) << "\n\n";
	}
	cout << "---********------*******-------\n\n\n" << endl;


	Logger &log = Logger::GetInstance();
	log.Log("Inverse Kinematics -- Yaw Rotation: " + to_string(IK_output.YawRotation * (180 / 3.14)) +
				" deg | Pitch Rotation: " + to_string(IK_output.PitchRotation * (180 / 3.14)) +
				" deg | Probe Rotation: " + to_string(IK_output.ProbeRotation * (180 / 3.14)) +
				" deg | Probe Insertion: " + to_string(IK_output.ProbeInsertion) +
				" mm | Lateral Translation: " + to_string(IK_output.LateralTranslation) +
				" mm | Axial Head Translation: " + to_string(IK_output.AxialHeadTranslation) +
				" mm |  Axial Feet Translation: " + to_string(IK_output.AxialFeetTranslation) + " mm",
			LOG_LEVEL_INFO, true);

	//_probe1 = { canulaToTreatment, treatmentToTip, robotToEntry, robotToTreatmentAtHome;}
	log.Log("Cannula Length -- " + to_string(_probe._robotToTreatmentAtHome + IK_output.ProbeInsertion - _probe._cannulaToTreatment) + " mm", LOG_LEVEL_INFO, true);

	// New Target Pose Calculation -- Check if Target Pose is Reachable from Axis Setpoint Validator
	Eigen::Matrix4d valid_fk = _neuroKinematics.ForwardKinematics(
												   validAxisSetpoints.at(0) / _axialHeadTranslation._ticksPerUnit, validAxisSetpoints.at(1) / _axialFeetTranslation._ticksPerUnit,
												   validAxisSetpoints.at(2) / _lateralTranslation._ticksPerUnit, validAxisSetpoints.at(3) / _probeInsertion._ticksPerUnit,
												   validAxisSetpoints.at(4) / _probeRotation._ticksPerUnit, validAxisSetpoints.at(5) / _pitchRotation._ticksPerUnit,
												   validAxisSetpoints.at(6) / _yawRotation._ticksPerUnit)
								   .zFrameToTreatment;

	_targetPose = _registration * valid_fk;

	// Print Desired and Reachable for Debugging purposes
	// TODO: Figure out how to log this instead later...
	cout << "Desired Target Pose" << endl;
	cout << _desiredTargetPose << endl;
	cout << "Reachable TargetPose" << endl;
	cout << _targetPose << endl;
}

/*
// This method checks if the a given set of IK outputs are valid
vector<int> NeuroRobot::Axis_Setpoint_Validator()
{
	Logger &log = Logger::GetInstance();

	// Define Valid Positions
	int validAxialHeadTranslation = _axialHeadTranslation._setpoint;
	int validAxialFeetTranslation = _axialFeetTranslation._setpoint;
	int validLateralTranslation = _lateralTranslation._setpoint;
	int validProbeInsertion = _probeInsertion._setpoint;
	int validProbeRotation = _probeRotation._setpoint;
	int validPitchRotation = _pitchRotation._setpoint;
	int validYawRotation = _yawRotation._setpoint;

	//------------------------------------------------------------------------
	// Lateral Validations
	if (_lateralTranslation._setpoint > _lateralTranslation._maxTicks)
	{
		log.Log("Lateral Translation of " + to_string(_lateralTranslation._setpoint / _lateralTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_lateralTranslation._maxTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validLateralTranslation = _lateralTranslation._maxTicks;
	}

	if (_lateralTranslation._setpoint < _lateralTranslation._minTicks)
	{
		log.Log("Lateral Translation of " + to_string(_lateralTranslation._setpoint / _lateralTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_lateralTranslation._minTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validLateralTranslation = _lateralTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Axial Distance Validations
	double axialSeparation = 143 + _axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit - _axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit;
	if (axialSeparation < 75.00)
	{
		log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the minimum allowed separation of 75 mm", LOG_LEVEL_WARNING, false);
	}

	// if( axialSeparation > 245){
	if (axialSeparation > 146)
	{
		log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the maximum allowed separation of  146 mm", LOG_LEVEL_WARNING, false);
	}

	//------------------------------------------------------------------------
	// Axial Head Validations
	if (_axialHeadTranslation._setpoint > _axialHeadTranslation._maxTicks)
	{
		log.Log("Axial Head Translation of " + to_string(_axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialHeadTranslation._maxTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialHeadTranslation = _axialHeadTranslation._maxTicks;
	}

	if (_axialHeadTranslation._setpoint < _axialHeadTranslation._minTicks)
	{
		log.Log("Axial Head Translation of " + to_string(_axialHeadTranslation._setpoint / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialHeadTranslation._minTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialHeadTranslation = _axialHeadTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Axial Feet Validations
	if (_axialFeetTranslation._setpoint > _axialFeetTranslation._maxTicks)
	{
		log.Log("Axial Feet Translation of " + to_string(_axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialFeetTranslation._maxTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialFeetTranslation = _axialFeetTranslation._maxTicks;
	}

	if (_axialFeetTranslation._setpoint < _axialFeetTranslation._minTicks)
	{
		log.Log("Axial Feet Translation of " + to_string(_axialFeetTranslation._setpoint / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialFeetTranslation._minTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialFeetTranslation = _axialFeetTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Probe Insertion Validations
	if (_probeInsertion._setpoint > _probeInsertion._maxTicks)
	{
		log.Log("Probe Insertion of " + to_string(_probeInsertion._setpoint / _probeInsertion._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_probeInsertion._maxTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validProbeInsertion = _probeInsertion._maxTicks;
	}

	if (_probeInsertion._setpoint < _probeInsertion._minTicks)
	{
		log.Log("Probe Insertion of " + to_string(_probeInsertion._setpoint / _probeInsertion._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_probeInsertion._minTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validProbeInsertion = _probeInsertion._minTicks;
	}

	//------------------------------------------------------------------------
	// Yaw Validations
	if (_yawRotation._setpoint > _yawRotation._maxTicks)
	{
		log.Log("Yaw Rotation of " + to_string(_yawRotation._setpoint / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_yawRotation._maxTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validYawRotation = _yawRotation._maxTicks;
	}

	if (_yawRotation._setpoint < _yawRotation._minTicks)
	{
		log.Log("Yaw Rotation of " + to_string(_yawRotation._setpoint / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_yawRotation._minTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validYawRotation = _yawRotation._minTicks;
	}

	//------------------------------------------------------------------------
	// Pitch Validations
	if (_pitchRotation._setpoint > _pitchRotation._maxTicks)
	{
		log.Log("Pitch Rotation of " + to_string(_pitchRotation._setpoint / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validPitchRotation = _pitchRotation._maxTicks;
	}

	if (_pitchRotation._setpoint < _pitchRotation._minTicks)
	{
		log.Log("Pitch Rotation of " + to_string(_pitchRotation._setpoint / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validPitchRotation = _pitchRotation._minTicks;
	}

	//------------------------------------------------------------------------
	// Probe Rotation Validations
	// TODO: Fill this in when ready

	// Return valid Axis Set points
	vector<int> validAxisSetpoints = {validAxialHeadTranslation, validAxialFeetTranslation, validLateralTranslation, validProbeInsertion, validProbeRotation, validPitchRotation, validYawRotation};
	return validAxisSetpoints;
}
*/


// This method checks if the a given set of IK outputs are valid
std::vector<int> NeuroRobot::Axis_Setpoint_Validator(std::vector<int> &input)
{
	Logger &log = Logger::GetInstance();

	// Define Valid Positions
	int validAxialHeadTranslation = input.at(0); //_axialHeadTranslation._setpoint;
	int validAxialFeetTranslation = input.at(1); //_axialFeetTranslation._setpoint;
	int validLateralTranslation = input.at(2);	 //_lateralTranslation._setpoint;
	int validProbeInsertion = input.at(3);		 //_probeInsertion._setpoint;
	int validProbeRotation = input.at(4);		 //_probeRotation._setpoint;
	int validPitchRotation = input.at(5);		 //_pitchRotation._setpoint;
	int validYawRotation = input.at(6);			 //_yawRotation._setpoint;

	//------------------------------------------------------------------------
	// Lateral Validations
	if (validLateralTranslation > _lateralTranslation._maxTicks)
	{
		log.Log("Lateral Translation of " + to_string(validLateralTranslation / _lateralTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_lateralTranslation._maxTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validLateralTranslation = _lateralTranslation._maxTicks;
	}

	else if (validLateralTranslation < _lateralTranslation._minTicks)
	{
		log.Log("Lateral Translation of " + to_string(validLateralTranslation / _lateralTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_lateralTranslation._minTicks / _lateralTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validLateralTranslation = _lateralTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Axial Distance Validations
	double axialSeparation = 143 + validAxialHeadTranslation / _axialHeadTranslation._ticksPerUnit - validAxialFeetTranslation / _axialFeetTranslation._ticksPerUnit;
	if (axialSeparation < 75.00)
	{
		log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the minimum allowed separation of 75 mm", LOG_LEVEL_WARNING, false);
	}

	// if( axialSeparation > 245){
	else if (axialSeparation > 146)
	{
		log.Log("Axial Separation of " + to_string(axialSeparation) + " mm  will exceed the maximum allowed separation of  146 mm", LOG_LEVEL_WARNING, false);
	}

	//------------------------------------------------------------------------
	// Axial Head Validations
	if (validAxialHeadTranslation > _axialHeadTranslation._maxTicks)
	{
		log.Log("Axial Head Translation of " + to_string(validAxialHeadTranslation / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialHeadTranslation._maxTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialHeadTranslation = _axialHeadTranslation._maxTicks;
	}

	else if (validAxialHeadTranslation < _axialHeadTranslation._minTicks)
	{
		log.Log("Axial Head Translation of " + to_string(validAxialHeadTranslation / _axialHeadTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialHeadTranslation._minTicks / _axialHeadTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialHeadTranslation = _axialHeadTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Axial Feet Validations
	if (validAxialFeetTranslation > _axialFeetTranslation._maxTicks)
	{
		log.Log("Axial Feet Translation of " + to_string(validAxialFeetTranslation / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_axialFeetTranslation._maxTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialFeetTranslation = _axialFeetTranslation._maxTicks;
	}

	else if (validAxialFeetTranslation < _axialFeetTranslation._minTicks)
	{
		log.Log("Axial Feet Translation of " + to_string(validAxialFeetTranslation / _axialFeetTranslation._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_axialFeetTranslation._minTicks / _axialFeetTranslation._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validAxialFeetTranslation = _axialFeetTranslation._minTicks;
	}

	//------------------------------------------------------------------------
	// Probe Insertion Validations
	if (validProbeInsertion > _probeInsertion._maxTicks)
	{
		log.Log("Probe Insertion of " + to_string(validProbeInsertion / _probeInsertion._ticksPerUnit) + " mm  will exceed the maximum allowed distance of " + to_string(_probeInsertion._maxTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validProbeInsertion = _probeInsertion._maxTicks;
	}

	else if (validProbeInsertion < _probeInsertion._minTicks)
	{
		log.Log("Probe Insertion of " + to_string(validProbeInsertion / _probeInsertion._ticksPerUnit) + " mm  will exceed the minimum allowed distance of " + to_string(_probeInsertion._minTicks / _probeInsertion._ticksPerUnit) + " mm", LOG_LEVEL_WARNING, false);
		validProbeInsertion = _probeInsertion._minTicks;
	}

	//------------------------------------------------------------------------
	// Yaw Validations
	if (validYawRotation > _yawRotation._maxTicks)
	{
		log.Log("Yaw Rotation of " + to_string(validYawRotation / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_yawRotation._maxTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validYawRotation = _yawRotation._maxTicks;
	}

	if (validYawRotation < _yawRotation._minTicks)
	{
		log.Log("Yaw Rotation of " + to_string(validYawRotation / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_yawRotation._minTicks / _yawRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validYawRotation = _yawRotation._minTicks;
	}

	//------------------------------------------------------------------------
	// Pitch Validations
	if (validPitchRotation > _pitchRotation._maxTicks)
	{
		log.Log("Pitch Rotation of " + to_string(validPitchRotation / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the maximum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validPitchRotation = _pitchRotation._maxTicks;
	}

	if (validPitchRotation < _pitchRotation._minTicks)
	{
		log.Log("Pitch Rotation of " + to_string(validPitchRotation / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees  will exceed the minimum allowed distance of " + to_string(_pitchRotation._maxTicks / _pitchRotation._ticksPerUnit * 180 / 3.14) + " degrees", LOG_LEVEL_WARNING, false);
		validPitchRotation = _pitchRotation._minTicks;
	}

	//------------------------------------------------------------------------
	// Probe Rotation Validations
	// TODO: Fill this in when ready

	// Return valid Axis Set points
	vector<int> validAxisSetpoints = {validAxialHeadTranslation, validAxialFeetTranslation, validLateralTranslation, validProbeInsertion, validProbeRotation, validPitchRotation, validYawRotation};
	return validAxisSetpoints;
}

/*
// Method that returns whether the footpedal is pressed or not
// This method is broken out like this because the footpedal is not an attribute of a Robot
// meaning it can not be accessed easily via the CustomWebServer
int NeuroRobot::IsFootPedalPressed()
{
	return _fpga_util->IsFootPedalPressed();
}

// Method that lists the motors specific to this robot
vector<Motor *> NeuroRobot::ListMotors()
{
	vector<Motor *> motors = {&_yawRotation,
							  &_probeRotation,
							  &_pitchRotation,
							  &_probeInsertion,
							  &_lateralTranslation,
							  &_axialHeadTranslation,
							  &_axialFeetTranslation};

	return motors;
}

// Method that returns a pointer to a single motor given its card slot id
Motor *NeuroRobot::GetMotor(int cardID)
{
	vector<Motor *> motors = ListMotors();
	for (Motor *motor : motors)
	{
		if (motor->_cardID == cardID)
		{
			return motor;
		}
	}

	return NULL;
}

// NeuroRobot specific method for following a given trajectory
void NeuroRobot::FollowTrajectory()
{

	// TODO: Reach set points in desired amount of time
	// TODO: Perform transformations and unit conversions to incoming trajectory as needed

	// If there are setpoints to follow in the trajectory
	Logger &log = Logger::GetInstance();

	// If we haven't reached the end of the trajectory
	if (_trajectoryIndex < _trajectory.size())
	{
		// If the current position is NOT the desired position, then travel to the desired positions
		if (
			!(_yawRotation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][1] - _yawRotation._deadband && _yawRotation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][1] + _yawRotation._deadband) ||
			!(_probeRotation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][2] - _probeRotation._deadband && _probeRotation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][2] + _probeRotation._deadband) ||
			!(_pitchRotation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][3] - _pitchRotation._deadband && _pitchRotation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][3] + _pitchRotation._deadband) ||
			!(_probeInsertion.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][4] - _probeInsertion._deadband && _probeInsertion.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][4] + _probeInsertion._deadband) ||
			!(_lateralTranslation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][5] - _lateralTranslation._deadband && _lateralTranslation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][5] + _lateralTranslation._deadband) ||
			!(_axialFeetTranslation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][6] - _axialFeetTranslation._deadband && _axialFeetTranslation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][6] + _axialFeetTranslation._deadband) ||
			!(_axialHeadTranslation.GetEncoderPositionTicks() >= _trajectory[_trajectoryIndex][7] - _axialHeadTranslation._deadband && _axialHeadTranslation.GetEncoderPositionTicks() <= _trajectory[_trajectoryIndex][7] + _axialHeadTranslation._deadband))
		{
			// Motor Set points are set to their current trajectory values
			_yawRotation._setpoint = _trajectory[_trajectoryIndex][1];
			_probeRotation._setpoint = _trajectory[_trajectoryIndex][2];
			_pitchRotation._setpoint = _trajectory[_trajectoryIndex][3];
			_probeInsertion._setpoint = _trajectory[_trajectoryIndex][4];
			_lateralTranslation._setpoint = _trajectory[_trajectoryIndex][5];
			_axialHeadTranslation._setpoint = _trajectory[_trajectoryIndex][6];
			_axialFeetTranslation._setpoint = _trajectory[_trajectoryIndex][7];

			// Motor Velocity set points
			double desiredPositionTicks = _trajectory[_trajectoryIndex][1];
			double desiredVelTicksPerSecond = _trajectory[_trajectoryIndex][8];
			//			_yawRotation.BangBang(desiredVelTicksPerSecond);

			if (_trajectoryIndex == 0)
			{
				// We are assuming the initial starting position for the first path is zero
				// TODO: We shouldn't assume for now that the initial starting position of the first trajectory is zero
				_yawRotation._velocity = _yawRotation.FollowTrapezoidalTrajectory(0, desiredPositionTicks, desiredVelTicksPerSecond);
			}
			else
			{
				_yawRotation._velocity = _yawRotation.FollowTrapezoidalTrajectory(_trajectory[_trajectoryIndex - 1][1], desiredPositionTicks, desiredVelTicksPerSecond);
			}

			log.Log("Trajectory -- Yaw Rotation: " + to_string(_yawRotation.GetEncoderPositionUnit() * (180 / 3.14)) +
						" deg | Pitch Rotation: " + to_string(_pitchRotation.GetEncoderPositionUnit() * (180 / 3.14)) +
						" deg | Probe Insertion: " + to_string(_probeInsertion.GetEncoderPositionUnit()) +
						" mm | Lateral Translation: " + to_string(_lateralTranslation.GetEncoderPositionUnit()) +
						" mm | Axial Head Translation: " + to_string(_axialHeadTranslation.GetEncoderPositionUnit()) +
						" mm |  Axial Feet Translation: " + to_string(_axialFeetTranslation.GetEncoderPositionUnit()) + " mm",
					LOG_LEVEL_INFO, true);
		}

		// If the current position is the desired position, then increment the trajectory index
		else
		{
			_trajectoryIndex += 1;
		}
	}
	else
	{
		log.Log("Trajectory Complete !", LOG_LEVEL_INFO, true);
		_yawRotation._velocity = _yawRotation.FollowTrapezoidalTrajectory(_trajectory[_trajectoryIndex - 1][1], _trajectory[_trajectoryIndex - 1][1], 0);
		//		_mode = "Manual"; // Lets go back to manual mode on completion
	}
}

// Neurorobot specific method that applies a range of frequencies to test for robot movement
void NeuroRobot::FrequencySweep(int cardID)
{
	Logger &log = Logger::GetInstance();
	// Frequency to start sweep is given by _frequency variable defined with a default value in constructor
	int end = 0x40000000; // Frequency to end sweep
	int step = 1000000;	  // Steps between frequencies

	Motor *motor = GetMotor(cardID);
	if (_frequency > end)
	{
		_timer.toc();
		motor->_velocity = _frequency;

		if (_timer.time() > 1000000)
		{
			_timer.tic();
			_frequency -= step;
			log.Log("Just Completed Frequency " + to_string(_frequency), LOG_LEVEL_INFO, true);
			cout << std::hex << _frequency << endl;
		}
	}
	else
	{
		log.Log("Frequency Sweep Complete !", LOG_LEVEL_INFO, true);
		_mode = "Manual";
		_frequency = 0x48000000; // Reset Frequency and Mode for next time in loop
	}
}
*/

// ==================================================================================
// NeuroRobot specific methods for coordinated control
// bool NeuroRobot::DetectMotorStateChange(){
//	// Compare with cached motors ?
//}
//
// void NeuroRobot::CoordinatedMove(vector<Motor *> motors){
//	for(Motor* motor:motors){
//		double subsetpoint = motor->_coordinated_move_subsetpoint;
//		int current = motor->GetEncoderPositionTicks();
//		int deadband = motor->_deadband;
//
//		if(subsetpoint < (current - deadband) || subsetpoint > (current + _deadband)){
//			motor->MoveMotor();
//		} else{
//			motor->_coordinated_move_subsetpoint_reached = true;
//			motor->StopMotor();
//		}
//	}
//}
//
// void NeuroRobot::CoordinatedControlCalculateIncrements(vector<Motor *> motors, double N){
//	for(Motor* motor:motors){
//		// Calculate Increment in Ticks
//		double desired = double(motor->_setpoint);
//		double current = motor->GetEncoderPositionTicks();
//		motor->_coordinated_move_motor_increment = abs(current - desired) / N;
//	}
//}
//
// bool NeuroRobot::AllMotorSubSetpointsReached(vector<Motor *> motors){
//	// Variable to determine if all axis are disabled
//	bool anyEnabled = false;
//
//	// Check if any enabled motors have not yet reached their sub set point
//	for(Motor* motor:motors){
//		if(motor->_enabled){
//			anyEnabled = true;
//			if(!motor->_coordinated_move_subsetpoint_reached){
//				return false;
//			}
//		}
//	}
//
//	return anyEnabled;
//}
//
// void NeuroRobot::CoordinatedControlUpdateSetpoints(vector<Motor *> motors){
//	for(Motor* motor:motors){
//		// Only increment sub set points of enabled motors
//		if(motor->_enabled){
//			// Coordinated Move Related Variables
//			int desired = motor->_setpoint;
//			double current_subsetpoint = motor->_coordinated_move_subsetpoint;
//
//			// Determine if we need to increase or decrease the sub set point by the motor increment
//			// if the current sub set point is less than the desired -- increase
//			if(current_subsetpoint < desired){
//				// Add N
//				current_subsetpoint += motor->_coordinated_move_motor_increment;
//
//				// If we have overshot the desired set point (by adding)
//				if(current_subsetpoint > desired){
//					currentsubsetpoint = desired;
//				}
//			}
//			// if the current sub set point is greater than the desired -- decrease
//			else if (current_subsetpoint > desired) {
//				// Subtract N
//				current_subsetpoint -= motor->_coordinated_move_motor_increment;
//
//				// If we have overshot the desired set point (by subtracting)
//				if(current_subsetpoint < desired){
//					currentsubsetpoint = desired;
//				}
//			}
//			// if the current sub set point is equal to the desired -- no change
//			else {
//				// If we are exactly equaly to the desired -- stay at the desired
//				current_subsetpoint = desired;
//			}
//
//			// update coordinated move sub set point
//			motor->_coordinated_move_subsetpoint = current_subsetpoint;
//
//		}
//	}
//}

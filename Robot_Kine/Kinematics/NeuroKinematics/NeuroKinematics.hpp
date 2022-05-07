//============================================================================
// Name        : NeuroKinematics.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#ifndef NEUROKINEMATICS_HPP_
#define NEUROKINEMATICS_HPP_

#include <math.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "../../Robots/Robot.hpp"
#include "../../Kinematics/Kinematics.hpp"
#include "eigen3/Eigen/Dense"

// TODO : Rename Prostate FK and IK structs following this convention
struct Neuro_FK_outputs
{
	Eigen::Matrix4d zFrameToTreatment;
};

struct Neuro_IK_outputs
{
	Eigen::Matrix4d targetPose;
	double AxialFeetTranslation;
	double AxialHeadTranslation;
	double LateralTranslation;
	double ProbeInsertion;
	double ProbeRotation;
	double YawRotation;
	double PitchRotation;
};

class NeuroKinematics : public Kinematics
{

public:
	//================ Constructor ================
	NeuroKinematics();
	NeuroKinematics(Probe *probe); /// This value comes from the robot probe

	//================ Parameters =================
	// Robot Specific Parameters
	double _lengthOfAxialTrapezoidSideLink;
	double _initialAxialSeperation;
	double _widthTrapezoidTop;
	double _xInitialRCM;
	double _yInitialRCM;
	double _zInitialRCM;
	double _robotToRCMOffset;

	Probe *_probe;				  // Object that stores probe specific configurations
	Eigen::Matrix4d _zFrameToRCM; // Transformation that accounts for change in rotation between zFrame and RCM

	// Parameters used in IK and FK calculations are moved out of their functions to keep
	// these methods running fast by eliminating the need to re-allocate memory every time
	Eigen::Matrix3d xRotationDueToYawRotationFK;
	Eigen::Matrix3d yRotationDueToPitchRotationFK;
	Eigen::Matrix3d zRotationDueToProbeRotationFK;
	Eigen::Matrix4d zFrameToRCMRotation;
	Eigen::Matrix4d zFrameToRCMPrime;
	Eigen::Matrix4d RCMToTreatment;

	Eigen::Matrix3d xRotationDueToYawRotationIK;
	Eigen::Matrix3d yRotationDueToPitchRotationIK;
	Eigen::Matrix3d zRotationDueToProbeRotationIK;
	Eigen::Matrix4d zFrameToTargetPointFinal;

	//================ Public Methods ==============
	Neuro_FK_outputs ForwardKinematics(double AxialHeadTranslation, double AxialFeetTranslation,
									   double LateralTranslation, double ProbeInsertion,
									   double ProbeRotation, double PitchRotation, double YawRotation);
	/*IK method for the case where vectors for the 3D location of the EP and TP with respect to the zFrame are provided*/
	Neuro_IK_outputs InverseKinematics(Eigen::Vector4d entryPointzFrame, Eigen::Vector4d targetPointzFrame);
	/*IK method for the case where full target pose is given (including rotation and position). EP is still a vector*/
	Neuro_IK_outputs InverseKinematics(Eigen::Vector4d entryPointzFrame, Eigen::Matrix4d _targetPointFullPoseScanner);
};

#endif /* NEUROKINEMATICS_HPP_ */

//============================================================================
// Name        : NeuroKinematics.hpp
// Author      : Produced in the WPI AIM Lab
// Description : This file is where the custom forward and inverse kinematics
//				 code for the robot resides
//============================================================================

#include "NeuroKinematics.hpp"
#define RADIAN_TO_DEGREE 57.29578

#include <iostream>
using namespace std;

NeuroKinematics::NeuroKinematics()
{
	_lengthOfAxialTrapezoidSideLink = 0;
	_widthTrapezoidTop = 0;
	_initialAxialSeperation = 0;
	_xInitialRCM = 0;
	_yInitialRCM = 0;
	_zInitialRCM = 0;
	_robotToRCMOffset = 0;

	// Object that stores probe specific configurations
	_probe = NULL;

	// Transformation that accounts for change in rotation between zFrame and RCM
	_zFrameToRCM = Eigen::Matrix4d::Identity();

	// Optimizations
	xRotationDueToYawRotationFK = Eigen::Matrix3d::Identity();
	yRotationDueToPitchRotationFK = Eigen::Matrix3d::Identity();
	zRotationDueToProbeRotationFK = Eigen::Matrix3d::Identity();
	zFrameToRCMRotation = Eigen::Matrix4d::Identity();
	zFrameToRCMPrime = Eigen::Matrix4d::Identity();
	RCMToTreatment = Eigen::Matrix4d::Identity();

	xRotationDueToYawRotationIK = Eigen::Matrix3d::Identity();
	yRotationDueToPitchRotationIK = Eigen::Matrix3d::Identity();
	zRotationDueToProbeRotationIK = Eigen::Matrix3d::Identity();
	zFrameToTargetPointFinal = Eigen::Matrix4d::Identity();
}

NeuroKinematics::NeuroKinematics(Probe *probe)
{
	// All values are in units of mm

	// Robot Link Measurements
	_lengthOfAxialTrapezoidSideLink = 60;
	_widthTrapezoidTop = 30;
	_initialAxialSeperation = 143;

	// X,Y,Z between the center of the Z-frame to robot RCM
	_xInitialRCM = -32.41; // Pre-Longer Links 18.32 - Length Change 50 = -31.68
	_yInitialRCM = 154.38; // Old 178.39 - Difference 21.85 = 156.54
	_zInitialRCM = 84.198; //Old Value 72.38;

	// Distance from needle driver to RCM
	_robotToRCMOffset = 72.5;

	// Object that stores probe specific configurations
	_probe = probe;

	// Transformation that accounts for change in rotation between zFrame and RCM
	_zFrameToRCM << -1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0, 0, 0, 0, 1;

	// Optimizations
	xRotationDueToYawRotationFK = Eigen::Matrix3d::Identity();
	yRotationDueToPitchRotationFK = Eigen::Matrix3d::Identity();
	zRotationDueToProbeRotationFK = Eigen::Matrix3d::Identity();
	zFrameToRCMRotation = Eigen::Matrix4d::Identity();
	zFrameToRCMPrime = Eigen::Matrix4d::Identity();
	RCMToTreatment = Eigen::Matrix4d::Identity();

	xRotationDueToYawRotationIK = Eigen::Matrix3d::Identity();
	yRotationDueToPitchRotationIK = Eigen::Matrix3d::Identity();
	zRotationDueToProbeRotationIK = Eigen::Matrix3d::Identity();
	zFrameToTargetPointFinal = Eigen::Matrix4d::Identity();
}

// This method defines the forward kinematics for the neurosurgery robot.
// Returns: A 4x4 transformation from the robot base to the robot treatment zone
// Note: All rotations should be given in units of radians
Neuro_FK_outputs NeuroKinematics::ForwardKinematics(double AxialHeadTranslation,
													double AxialFeetTranslation, double LateralTranslation,
													double ProbeInsertion, double ProbeRotation, double PitchRotation,
													double YawRotation)
{
	// Structure to return with the FK output
	struct Neuro_FK_outputs FK;

	// Z position of RCM is solely defined as the midpoint of the axial trapezoid
	double axialTrapezoidMidpoint = (AxialHeadTranslation - AxialFeetTranslation + _initialAxialSeperation) / 2;
	double zDeltaRCM = (AxialFeetTranslation + AxialHeadTranslation) / 2;

	// Y position of RCM is found by pythagorean theorem of the axial trapezoid
	double yTrapezoidHypotenuseSquared = pow(_lengthOfAxialTrapezoidSideLink,
											 2);
	double yTrapezoidSideSquared = pow(
		(axialTrapezoidMidpoint - _widthTrapezoidTop / 2), 2);
	double yTrapezoidInitialSeparationSquared = pow(
		(_initialAxialSeperation - _widthTrapezoidTop) / 2, 2);
	double yDeltaRCM = sqrt(yTrapezoidHypotenuseSquared - yTrapezoidSideSquared) - sqrt(
																					   yTrapezoidHypotenuseSquared - yTrapezoidInitialSeparationSquared);

	// X position of RCM is solely defined as the amount traveled in lateral translation
	double xDeltaRCM = LateralTranslation;

	// Obtain basic Yaw, Pitch, and Roll Rotations
	xRotationDueToYawRotationFK << 1, 0, 0, 0, cos(YawRotation), -sin(YawRotation), 0, sin(YawRotation), cos(YawRotation);

	yRotationDueToPitchRotationFK << cos(PitchRotation), 0, sin(PitchRotation), 0, 1, 0, -sin(PitchRotation), 0, cos(PitchRotation);

	zRotationDueToProbeRotationFK << cos(ProbeRotation), -sin(ProbeRotation), 0, sin(ProbeRotation), cos(ProbeRotation), 0, 0, 0, 1;

	// Calculate the XYZ Rotation
	zFrameToRCMRotation.block(0, 0, 3, 3) =
		(xRotationDueToYawRotationFK * yRotationDueToPitchRotationFK * zRotationDueToProbeRotationFK).block(0, 0, 3, 3);

	// Calculate the XYZ Translation
	zFrameToRCMPrime << -1, 0, 0, _xInitialRCM + xDeltaRCM, 0, 0, -1, _yInitialRCM + yDeltaRCM, 0, -1, 0, _zInitialRCM + zDeltaRCM, 0, 0, 0, 1;

	// Now Calculate zFrame to RCM given the calculated values above
	Eigen::Matrix4d zFrameToRCM = zFrameToRCMPrime * zFrameToRCMRotation;

	// Create RCM to Treatment Matrix
	RCMToTreatment(2, 3) = ProbeInsertion + _probe->_robotToTreatmentAtHome - _robotToRCMOffset;

	// Finally calculate Base to Treatment zone using the measured transformation for RCM to Treatment
	FK.zFrameToTreatment = zFrameToRCM * RCMToTreatment;

	return FK;
}

// This method defines the inverse kinematics for the neurosurgery robot
// Given: Vectors for the 3D location of the entry point and target point with respect to the zFrame
// Returns: The joint values for the given approach
Neuro_IK_outputs NeuroKinematics::InverseKinematics(
	Eigen::Vector4d entryPointzFrame, Eigen::Vector4d targetPointzFrame)
{

	// Structure to return the results of the IK
	struct Neuro_IK_outputs IK;

	// Get the entry point with respect to the orientation of the zFrame
	Eigen::Vector4d rcmToEntry = _zFrameToRCM.inverse() * entryPointzFrame;

	// Get the target point with respect to the orientation of the zFrame
	Eigen::Vector4d rcmToTarget = _zFrameToRCM.inverse() * targetPointzFrame;

	// Calculate Needle Vector
	Eigen::Vector4d needleVector = rcmToTarget - rcmToEntry;

	// Calculate Unit Vector of Needle
	Eigen::Vector4d needleUnitVector = needleVector / needleVector.norm();

	// Calculate Yaw and Pitch
	IK.YawRotation = atan2(-needleUnitVector(1), needleUnitVector(2));
	IK.PitchRotation = asin(needleUnitVector(0));

	// TODO: Add IK for probe Rotation
	IK.ProbeRotation = 0;

	// ==========================================================================================================

	// The Translational elements on the Inverse Kinematics rely on the final location of the target point
	double XEntry = entryPointzFrame(0);
	double YEntry = entryPointzFrame(1);
	double ZEntry = entryPointzFrame(2);

	// The Lateral Translation is given by the desired distance in x
	IK.LateralTranslation = XEntry - _xInitialRCM - _robotToRCMOffset * sin(IK.PitchRotation) + _probe->_robotToEntry * sin(IK.PitchRotation);
	double NewLateralTranslation = XEntry - _xInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(0);

	// Old and New Here refer to with and without including the needle unit vector respectively
	// Both result in the same answer, so we opted to use the old version but should be updated to the new later
	cout << "Lateral Old and New" << endl;
	cout << IK.LateralTranslation << endl;
	cout << NewLateralTranslation << endl;

	// Equations calculated through the symbolic equations for the Forward Kinematics
	// Substituting known values in the FK equations yields the value for Axial Head and Feet
	IK.AxialHeadTranslation =
		ZEntry - _initialAxialSeperation / 2 + _widthTrapezoidTop / 2 - _zInitialRCM + sqrt(8 * YEntry * _yInitialRCM - 2 * _initialAxialSeperation * _widthTrapezoidTop - 4 * YEntry * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) + 4 * _yInitialRCM * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(_initialAxialSeperation, 2) + pow(_widthTrapezoidTop, 2) - 4 * pow(_yInitialRCM, 2) - 4 * pow(_robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * _robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * _robotToRCMOffset * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2))) / 2 + _robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);
	IK.AxialFeetTranslation =
		ZEntry + _initialAxialSeperation / 2 - _widthTrapezoidTop / 2 - _zInitialRCM - sqrt(8 * YEntry * _yInitialRCM - 2 * _initialAxialSeperation * _widthTrapezoidTop - 4 * YEntry * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) + 4 * _yInitialRCM * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(_initialAxialSeperation, 2) + pow(_widthTrapezoidTop, 2) - 4 * pow(_yInitialRCM, 2) - 4 * pow(_robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * _robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * _robotToRCMOffset * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2))) / 2 + _robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);

	double AxialZShift = ZEntry - _zInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(1);
	double AxialYShift = YEntry - _yInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(2);
	double AxialShiftDueToY = sqrt(
		pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(AxialYShift + 20.19, 2));

	double NewAxialHeadTranslation = AxialZShift - _initialAxialSeperation / 2 + _widthTrapezoidTop / 2 + AxialShiftDueToY;
	double NewAxialFeetTranslation = AxialZShift + _initialAxialSeperation / 2 - _widthTrapezoidTop / 2 - AxialShiftDueToY;

	// Old and New Here refer to with and without including the needle unit vector respectively
	// Both result in the same answer, so we opted to use the old version but should be updated to the new later
	cout << "Axial Old and New" << endl;
	cout << IK.AxialHeadTranslation << endl;
	cout << IK.AxialFeetTranslation << endl;
	cout << NewAxialHeadTranslation << endl;
	cout << NewAxialFeetTranslation << endl;

	// Probe Insertion is calculate as the distance between the entry point and the target point in 3D space (with considerations for the final treatment zone of the probe)
	IK.ProbeInsertion = sqrt(
							pow(entryPointzFrame(0) - targetPointzFrame(0), 2) + pow(entryPointzFrame(1) - targetPointzFrame(1), 2) + pow(entryPointzFrame(2) - targetPointzFrame(2), 2)) -
						_probe->_robotToTreatmentAtHome + _probe->_robotToEntry;

	// Obtain basic Yaw, Pitch, and Roll Rotations
	xRotationDueToYawRotationIK << 1, 0, 0, 0, cos(IK.YawRotation), -sin(IK.YawRotation), 0, sin(IK.YawRotation), cos(IK.YawRotation);

	yRotationDueToPitchRotationIK << cos(IK.PitchRotation), 0, sin(IK.PitchRotation), 0, 1, 0, -sin(IK.PitchRotation), 0, cos(IK.PitchRotation);

	zRotationDueToProbeRotationIK << cos(IK.ProbeRotation), -sin(IK.ProbeRotation), 0, sin(IK.ProbeRotation), cos(IK.ProbeRotation), 0, 0, 0, 1;
	// Calculate the XYZ Rotation
	zFrameToTargetPointFinal.block(0, 0, 3, 3) =
		(xRotationDueToYawRotationIK * yRotationDueToPitchRotationIK * zRotationDueToProbeRotationIK).block(0, 0, 3, 3);
	zFrameToTargetPointFinal = _zFrameToRCM * zFrameToTargetPointFinal;

	// The X,Y,Z location is already given by the target point
	zFrameToTargetPointFinal(0, 3) = targetPointzFrame(0);
	zFrameToTargetPointFinal(1, 3) = targetPointzFrame(1);
	zFrameToTargetPointFinal(2, 3) = targetPointzFrame(2);

	// Obtain the FK of the target Pose
	IK.targetPose = zFrameToTargetPointFinal;

	return IK;
}

// IK method for the case where target pose is given. (EP is still a coordinate point)
Neuro_IK_outputs NeuroKinematics::InverseKinematics(
	Eigen::Vector4d entryPointzFrame,
	Eigen::Matrix4d _targetPointFullPoseZframe)
{
	Eigen::Vector4d targetPointzFrame;
	targetPointzFrame.setOnes();
	// extracting the position element of the TP
	for (int i = 0; i < 3; i++)
	{
		targetPointzFrame(i) = _targetPointFullPoseZframe(i, 3);
	}

	// Structure to return the results of the IK
	struct Neuro_IK_outputs IK;

	// Get the entry point with respect to the orientation of the zFrame
	Eigen::Vector4d rcmToEntry = _zFrameToRCM.inverse() * entryPointzFrame;

	// Get the target point with respect to the orientation of the zFrame
	Eigen::Vector4d rcmToTarget = _zFrameToRCM.inverse() * targetPointzFrame;

	Eigen::Matrix4d rcmToTargetFullPose = _zFrameToRCM.inverse() * _targetPointFullPoseZframe;
//	cout << "\nCalculated Target Pose w.r.t RCM!" << endl;
//	cout << rcmToTargetFullPose(0, 0) << " , "
//		 << rcmToTargetFullPose(0, 1) << " , "
//		 << rcmToTargetFullPose(0, 2) << endl;
//	cout << rcmToTargetFullPose(1, 0) << " , "
//		 << rcmToTargetFullPose(1, 1) << " , "
//		 << rcmToTargetFullPose(1, 2) << endl;
//	cout << rcmToTargetFullPose(2, 0) << " , "
//		 << rcmToTargetFullPose(2, 1) << " , "
//		 << rcmToTargetFullPose(2, 2) << endl;

	// Calculate Needle Vector
	Eigen::Vector4d needleVector = rcmToTarget - rcmToEntry;

	// Calculate Unit Vector of Needle
	Eigen::Vector4d needleUnitVector = needleVector / needleVector.norm();

	// Check the needle vector with the approach vector of the sent target pose
	// For now it's just a sanity check to make sure that the TheraVision is sending the correct transform.
	Eigen::Vector4d rcmToTargetApproachVector = rcmToTargetFullPose.block(0, 2, 4, 1);

	if (!needleUnitVector.isApprox(rcmToTargetApproachVector, 1e-4))
	{
		cout << "\nThe approach vector does not match the desired target's z-axis!\n";
		cout << needleUnitVector << endl;
		cout << rcmToTargetApproachVector << endl;
	}

	// Calculate Yaw and Pitch (Old code)
	// IK.YawRotation = atan2(-needleUnitVector(1), needleUnitVector(2));
	// IK.PitchRotation = asin(needleUnitVector(0));
	double YawRotation_old = atan2(-needleUnitVector(1), needleUnitVector(2)); // remove after validation
	double PitchRotation_old = asin(needleUnitVector(0));					   // remove after validation

	double PitchRotation_sin = asin(rcmToTargetFullPose(0, 2)); // remove after validation

	IK.PitchRotation = atan2(rcmToTargetFullPose(0, 2),
							 sqrt(pow(rcmToTargetFullPose(1, 2), 2) + pow(rcmToTargetFullPose(2, 2), 2)));
	IK.ProbeRotation = atan2(-rcmToTargetFullPose(0, 1), rcmToTargetFullPose(0, 0));
	IK.YawRotation = atan2(-rcmToTargetFullPose(1, 2), rcmToTargetFullPose(2, 2));


	// Print the old and new Pitch, Yaw, ProbeRotation
//	cout << "\nProbe Rotation is(deg):" << IK.ProbeRotation * RADIAN_TO_DEGREE << endl;

	// ==========================================================================================================

	// The Translational elements on the Inverse Kinematics rely on the final location of the target point
	double XEntry = entryPointzFrame(0);
	double YEntry = entryPointzFrame(1);
	double ZEntry = entryPointzFrame(2);

	// The Lateral Translation is given by the desired distance in x
	IK.LateralTranslation = XEntry - _xInitialRCM - _robotToRCMOffset * sin(IK.PitchRotation) + _probe->_robotToEntry * sin(IK.PitchRotation);
	double NewLateralTranslation = XEntry - _xInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(0);

	// Old and New Here refer to with and without including the needle unit vector respectively
	// Both result in the same answer, so we opted to use the old version but should be updated to the new later
//	cout << "Lateral Old and New" << endl;
//	cout << IK.LateralTranslation << endl;
//	cout << NewLateralTranslation << endl;

	// Equations calculated through the symbolic equations for the Forward Kinematics
	// Substituting known values in the FK equations yields the value for Axial Head and Feet
	IK.AxialHeadTranslation =
		ZEntry - _initialAxialSeperation / 2 + _widthTrapezoidTop / 2 - _zInitialRCM + sqrt(8 * YEntry * _yInitialRCM - 2 * _initialAxialSeperation * _widthTrapezoidTop - 4 * YEntry * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) + 4 * _yInitialRCM * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(_initialAxialSeperation, 2) + pow(_widthTrapezoidTop, 2) - 4 * pow(_yInitialRCM, 2) - 4 * pow(_robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * _robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * _robotToRCMOffset * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2))) / 2 + _robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);
	IK.AxialFeetTranslation =
		ZEntry + _initialAxialSeperation / 2 - _widthTrapezoidTop / 2 - _zInitialRCM - sqrt(8 * YEntry * _yInitialRCM - 2 * _initialAxialSeperation * _widthTrapezoidTop - 4 * YEntry * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) + 4 * _yInitialRCM * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * pow(YEntry, 2) + pow(_initialAxialSeperation, 2) + pow(_widthTrapezoidTop, 2) - 4 * pow(_yInitialRCM, 2) - 4 * pow(_robotToRCMOffset, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) - 4 * pow(_probe->_robotToEntry, 2) * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * _robotToRCMOffset * _probe->_robotToEntry * pow(cos(IK.PitchRotation), 2) * pow(cos(IK.YawRotation), 2) + 8 * YEntry * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * YEntry * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) - 8 * _robotToRCMOffset * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 8 * _probe->_robotToEntry * _yInitialRCM * cos(IK.PitchRotation) * cos(IK.YawRotation) + 4 * _robotToRCMOffset * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2)) - 4 * _probe->_robotToEntry * cos(IK.PitchRotation) * cos(IK.YawRotation) * sqrt(-pow(_initialAxialSeperation, 2) + 2 * _initialAxialSeperation * _widthTrapezoidTop + 4 * pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(_widthTrapezoidTop, 2))) / 2 + _robotToRCMOffset * cos(IK.PitchRotation) * sin(IK.YawRotation) - _probe->_robotToEntry * cos(IK.PitchRotation) * sin(IK.YawRotation);

	double AxialZShift = ZEntry - _zInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(1);
	double AxialYShift = YEntry - _yInitialRCM + (-_robotToRCMOffset + _probe->_robotToEntry) * needleUnitVector(2);
	double AxialShiftDueToY = sqrt(
		pow(_lengthOfAxialTrapezoidSideLink, 2) - pow(AxialYShift + 20.19, 2));

	double NewAxialHeadTranslation = AxialZShift - _initialAxialSeperation / 2 + _widthTrapezoidTop / 2 + AxialShiftDueToY;
	double NewAxialFeetTranslation = AxialZShift + _initialAxialSeperation / 2 - _widthTrapezoidTop / 2 - AxialShiftDueToY;

	// Old and New Here refer to with and without including the needle unit vector respectively
	// Both result in the same answer, so we opted to use the old version but should be updated to the new later
//	cout << "Axial Old and New" << endl;
//	cout << IK.AxialHeadTranslation << endl;
//	cout << IK.AxialFeetTranslation << endl;
//	cout << NewAxialHeadTranslation << endl;
//	cout << NewAxialFeetTranslation << endl;

	// Probe Insertion is calculate as the distance between the entry point and the target point in 3D space (with considerations for the final treatment zone of the probe)
	IK.ProbeInsertion = sqrt(
							pow(entryPointzFrame(0) - targetPointzFrame(0), 2) + pow(entryPointzFrame(1) - targetPointzFrame(1), 2) + pow(entryPointzFrame(2) - targetPointzFrame(2), 2)) -
						_probe->_robotToTreatmentAtHome + _probe->_robotToEntry;

	// Obtain basic Yaw, Pitch, and Roll Rotations
	xRotationDueToYawRotationIK << 1, 0, 0, 0, cos(IK.YawRotation), -sin(IK.YawRotation), 0, sin(IK.YawRotation), cos(IK.YawRotation);

	yRotationDueToPitchRotationIK << cos(IK.PitchRotation), 0, sin(IK.PitchRotation), 0, 1, 0, -sin(IK.PitchRotation), 0, cos(IK.PitchRotation);

	zRotationDueToProbeRotationIK << cos(IK.ProbeRotation), -sin(IK.ProbeRotation), 0, sin(IK.ProbeRotation), cos(IK.ProbeRotation), 0, 0, 0, 1;

	// Calculate the XYZ Rotation
	zFrameToTargetPointFinal.block(0, 0, 3, 3) =
		(xRotationDueToYawRotationIK * yRotationDueToPitchRotationIK * zRotationDueToProbeRotationIK).block(0, 0, 3, 3);
	zFrameToTargetPointFinal = _zFrameToRCM * zFrameToTargetPointFinal;

	// The X,Y,Z location is already given by the target point
	zFrameToTargetPointFinal(0, 3) = targetPointzFrame(0);
	zFrameToTargetPointFinal(1, 3) = targetPointzFrame(1);
	zFrameToTargetPointFinal(2, 3) = targetPointzFrame(2);


//	cout << "----------Old Joint Values vs New Joint Values----------" << endl;
//	cout << "[OLD] AxialHeadTranslation Translation" << IK.AxialHeadTranslation << " [mm]" << endl;
//	cout << "[NEW] AxialHeadTranslation Translation" << NewAxialHeadTranslation << " [mm]" << endl << endl;
//
//	cout << "[OLD] AxialFeetTranslation Translation" << IK.AxialFeetTranslation << " [mm]" << endl;
//	cout << "[NEW] AxialFeetTranslation Translation" << NewAxialFeetTranslation << " [mm]" << endl << endl;
//
//	cout << "[OLD] LateralTranslation" << IK.LateralTranslation << " [mm]" << endl;
//	cout << "[NEW] LateralTranslation" << NewLateralTranslation << " [mm]" << endl << endl;
//
//	cout << "[OLD] PitchRotation" << PitchRotation_old << " [rad]" << endl;
//	cout << "[NEW] PitchRotation" << IK.PitchRotation << " [rad]" << endl << endl;
//
//	cout << "[OLD] YawRotation" << YawRotation_old << " [rad]" << endl;
//	cout << "[NEW] YawRotation" << IK.YawRotation << " [rad]" << endl << endl;
//
//	cout << "[OLD] ProbeInsertion" << entryPointzFrame(2,1) << " [mm]" << endl;
//	cout << "[NEW] ProbeInsertion" << _targetPointFullPoseZframe(2,3) << " [mm]" << endl << endl;
//
//	cout << "[OLD] ProbeRotation" << IK.ProbeRotation * RADIAN_TO_DEGREE << " [deg]" << endl;
//	cout << "[NEW] ProbeRotation" << IK.ProbeRotation * RADIAN_TO_DEGREE << " [deg]" << endl << endl;

/*
	// Compare the Desired Target pose vs. the calculated Target Pose
	cout << "\nThe Desired Target Pose!" << endl;
	cout << _targetPointFullPoseZframe(0, 0) << " , "
		 << _targetPointFullPoseZframe(0, 1) << " , "
		 << _targetPointFullPoseZframe(0, 2) << endl;
	cout << _targetPointFullPoseZframe(1, 0) << " , "
		 << _targetPointFullPoseZframe(1, 1) << " , "
		 << _targetPointFullPoseZframe(1, 2) << endl;
	cout << _targetPointFullPoseZframe(2, 0) << " , "
		 << _targetPointFullPoseZframe(2, 1) << " , "
		 << _targetPointFullPoseZframe(2, 2) << endl;
	cout << "\nCalculated Target Pose!" << endl;
	cout << zFrameToTargetPointFinal(0, 0) << " , "
		 << zFrameToTargetPointFinal(0, 1) << " , "
		 << zFrameToTargetPointFinal(0, 2) << endl;
	cout << zFrameToTargetPointFinal(1, 0) << " , "
		 << zFrameToTargetPointFinal(1, 1) << " , "
		 << zFrameToTargetPointFinal(1, 2) << endl;
	cout << zFrameToTargetPointFinal(2, 0) << " , "
		 << zFrameToTargetPointFinal(2, 1) << " , "
		 << zFrameToTargetPointFinal(2, 2) << endl;
 */
//	cout << "----------Desired Target Pose vs Calculated Target Pose----------" << endl;
//	cout << "Desired Target Pose: " << endl;
//	cout << _targetPointFullPoseZframe << endl;
//
//	cout << "Calculated Target Pose: " << endl;
//	cout << zFrameToTargetPointFinal << endl;
//	cout << "-----------------------------------------------------------------" << endl;

	IK.targetPose = zFrameToTargetPointFinal;
	return IK;
}

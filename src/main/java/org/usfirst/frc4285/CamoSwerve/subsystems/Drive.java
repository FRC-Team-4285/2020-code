/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import java.util.Arrays;
import java.util.Collections;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc4285.CamoSwerve.RobotMap;
import org.usfirst.frc4285.CamoSwerve.commands.FieldCentricSwerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Drive extends Subsystem {

	private CANSparkMax driveLeftFrontSpark;
	private CANSparkMax driveLeftRearSpark;
	private CANSparkMax driveRightFrontSpark;
  private CANSparkMax driveRightRearSpark;
  
	private TalonSRX steerLeftFront;
	private TalonSRX steerLeftRear;
	private TalonSRX steerRightFront;
	private TalonSRX steerRightRear;

  //Wheelbase lengthes are measured from where wheels touch the ground
	public static final double WHEEL_BASE_LENGTH = 24;  
  public static final double WHEEL_BASE_WIDTH = 24; 
  //Encoder counts are 1024 for ma3. 4096 for ctre mag encoders
	public static final double ENCODER_COUNT_PER_ROTATION = 1024; 

	public static final double WHEEL_DIAMETER = 4.0;
	//TODO: increase MAX_SPEED
	public static final double MAX_SPEED = 0.3; //Max speed is 0 to 1 
  public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
  //Drive inches per count is calculated for cimcoders under motor with a final gear reduction of 6.67
  //which is the reduction for the andymark swerve and steer.
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

  //The steering PIDs need to be adjusted for your drive. Start with I = D =0
  //Set P low and try moving. If no oscilation double P. When steer oscillates 
  //set P to last value that did not oscillate. set D to about P * 10 to start
	private static final double STEER_P = 1.0, STEER_I = 0.0, STEER_D = 0.0;
	private static final int STATUS_FRAME_PERIOD = 5;

	public Drive() {
    
    //BEGIN CONFIGURURATION OF DRIVE MOTOR CONTROLLERS

		driveLeftFrontSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
		driveLeftFrontSpark.restoreFactoryDefaults();
		driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
		driveLeftFrontSpark.setOpenLoopRampRate(0.125);
		driveLeftFrontSpark.setSmartCurrentLimit(60);
		
		driveLeftRearSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_REAR_ID, MotorType.kBrushless);
		driveLeftRearSpark.restoreFactoryDefaults();
		driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
		driveLeftRearSpark.setOpenLoopRampRate(0.125);
		driveLeftRearSpark.setSmartCurrentLimit(60);

		driveRightFrontSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
		driveRightFrontSpark.restoreFactoryDefaults();
		driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
		driveRightFrontSpark.setOpenLoopRampRate(0.125);
		driveRightFrontSpark.setSmartCurrentLimit(60);

		driveRightRearSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_REAR_ID, MotorType.kBrushless);
		driveRightRearSpark.restoreFactoryDefaults();
		driveRightRearSpark.setIdleMode(IdleMode.kBrake);
		driveRightRearSpark.setOpenLoopRampRate(0.125);
		driveRightRearSpark.setSmartCurrentLimit(60);
		// END OF DRIVE MOTOR CONTROLLER CONFIGURATION

		// BEGIN STEERING MOTOR CONTROLLER CONFIGURATION CODE
		steerLeftFront = new TalonSRX(RobotMap.STEER_LEFT_FRONT_ID);
		steerLeftFront.configFactoryDefault();
		steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerLeftFront.configFeedbackNotContinuous(false, 10);
    steerLeftFront.setSensorPhase(false);
    steerLeftFront.setInverted(false);
    steerLeftFront.config_kP(0, STEER_P, 10);
    steerLeftFront.config_kI(0, STEER_I, 10);
    steerLeftFront.config_kD(0, STEER_D, 10);
    steerLeftFront.config_IntegralZone(0, 100, 0);
    steerLeftFront.configAllowableClosedloopError(0, 5, 0);
		steerLeftFront.setNeutralMode(NeutralMode.Brake);
    steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerLeftRear = new TalonSRX(RobotMap.STEER_LEFT_REAR_ID);
		steerLeftRear.configFactoryDefault();
		steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerLeftRear.configFeedbackNotContinuous(false, 10);
    steerLeftRear.setSensorPhase(false);
    steerLeftRear.setInverted(false);
    steerLeftRear.config_kP(0, STEER_P, 10);
    steerLeftRear.config_kI(0, STEER_I, 10);
    steerLeftRear.config_kD(0, STEER_D, 10);
    steerLeftRear.config_IntegralZone(0, 100, 0);
    steerLeftRear.configAllowableClosedloopError(0, 5, 0);
    steerLeftRear.setNeutralMode(NeutralMode.Brake);
    steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightFront = new TalonSRX(RobotMap.STEER_RIGHT_FRONT_ID);
		steerRightFront.configFactoryDefault();
		steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerRightFront.configFeedbackNotContinuous(false, 10);
    steerRightFront.setSensorPhase(false);
    steerRightFront.setInverted(false);
    steerRightFront.config_kP(0, STEER_P, 10);
    steerRightFront.config_kI(0, STEER_I, 10);
    steerRightFront.config_kD(0, STEER_D, 10);
    steerRightFront.config_IntegralZone(0, 100, 0);
    steerRightFront.configAllowableClosedloopError(0, 5, 0);
    steerRightFront.setNeutralMode(NeutralMode.Brake);
    steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightRear = new TalonSRX(RobotMap.STEER_RIGHT_REAR_ID);
		steerRightRear.configFactoryDefault();
		steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerRightRear.configFeedbackNotContinuous(false, 10);
    steerRightRear.setSensorPhase(false);
    steerRightRear.setInverted(false);
    steerRightRear.config_kP(0, STEER_P, 10);
    steerRightRear.config_kI(0, STEER_I, 10);
    steerRightRear.config_kD(0, STEER_D, 10);
    steerRightRear.config_IntegralZone(0, 100, 0);
    steerRightRear.configAllowableClosedloopError(0, 5, 0);
    steerRightRear.setNeutralMode(NeutralMode.Brake);
    steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    // END OF STEERING MOTOR CONTROLLER CONFIGURATION
  }
  

	public void swerveDrive(double strafe, double forward, double omega) {
    double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
    
    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;
    
    // Compute the drive motor speeds
    double speedLF = speed(B, D);
    double speedLR = speed(A, D);
    double speedRF = speed(B, C);
    double speedRR = speed(A, C);
    
		// ... and angles for the steering motors 
		// If drives are calibrated for zero position on encoders they are at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
    // for initial position/calibration of drives.
    //This is only needed if the drives are calibrated to zero facing the sides
    //instead of front back.

		// double angleLF = angle(B, D) + 90;
    // double angleLR = angle(A, D) - 90;
    // double angleRF = angle(B, C) + 90;
    // double angleRR = angle(A, C) - 90;

    //If the drive are set to zero facing the front use the following
    double angleLF = angle(B, D);
    double angleLR = angle(A, D);
    double angleRF = angle(B, C);
    double angleRR = angle(A, C);
    // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    setSwerveModule(steerLeftFront, driveLeftFrontSpark, angleLF, speedLF / maxSpeed);
    setSwerveModule(steerLeftRear, driveLeftRearSpark, angleLR, speedLR / maxSpeed);
    setSwerveModule(steerRightFront, driveRightFrontSpark, angleRF, speedRF / maxSpeed);
		setSwerveModule(steerRightRear, driveRightRearSpark, angleRR, speedRR / maxSpeed);
	}
	
	private double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }
  
  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));
  }
	

	private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed) {
    double currentPosition = steer.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    // The angle from the encoder is in the range [0, 360], but the swerve computations
    // return angles in the range [-180, 180], so transform the encoder angle to this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    // TODO: Properly invert the steering motors so this isn't necessary
    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		
		//if (Math.abs(speed) <= MAX_SPEED){
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
    //drive.set(ControlMode.PercentOutput, speed);
    drive.set(speed);

	}

	//get Encoder values
	
	public double getSteerLFEncoder() {
		return steerLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerLREncoder() {
		return steerLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getSteerRFEncoder() {
		return steerRightFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerRREncoder() {
		return steerRightRear.getSelectedSensorPosition(0);
	}

	// //setting motors
	public void setDriveLeftFront(double speed){
		driveLeftFrontSpark.set(speed);
	}

	public void setDriveLeftRear(double speed){
		driveLeftRearSpark.set(speed);
	}
	
	public void setDriveRightFront(double speed){
		driveRightFrontSpark.set(speed);
	}
	
	public void setDriveRightRear(double speed){
		driveRightRearSpark.set(speed);
	}

	
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new FieldCentricSwerveDrive());
  }
	
}
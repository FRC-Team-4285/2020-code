/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import org.usfirst.frc4285.CamoSwerve.RobotMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.usfirst.frc4285.CamoSwerve.subsystems.Drive;
import org.usfirst.frc4285.CamoSwerve.OI;
import org.usfirst.frc4285.CamoSwerve.subsystems.Drive;


/**
 * Add your docs here.
 */
public class Turret extends Subsystem {

  private CANSparkMax turretmotor;
  private CANPIDController turretPID;
  private CANEncoder turretencoder;
  private NetworkTable table;
  private double a1;
  private double a2;
  private double h1;
  private double h2;
  private double d;
  private double power;
  private double leadAngle;
  private double swerveAverageRPM;
  private double robotSpeed;

  private CANEncoder driveLeftFrontEncoder;
  private CANEncoder driveLeftRearEncoder;
  private CANEncoder driveRightFrontEncoder;
  private CANEncoder driveRightRearEncoder;

	private CANSparkMax driveLeftFrontSpark;
	private CANSparkMax driveLeftRearSpark;
	private CANSparkMax driveRightFrontSpark;
  private CANSparkMax driveRightRearSpark;
  
  private double distanceCoefficiant;



  public Turret() {
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void following() {
    NetworkTableEntry tx = table.getEntry("tx");

    // Read values periodically
    //double x = tx.getDouble(0.0);
    double x = tx.getDouble(0.0);


    final NetworkTableEntry ty = table.getEntry("ty");
    a1 = 21; // Angel of camera from the horizontal in degrees
    a2 = ty.getDouble(0.0); // Angel of tower to camera found with limelight
    h1 = 27; // Height of limelight to ground in inches
    h2 = 82; // Height of tower's tape to ground in inches

    driveLeftFrontSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
		driveLeftFrontSpark.restoreFactoryDefaults();
    driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
    driveLeftFrontSpark.setInverted(false);
		driveLeftFrontSpark.setOpenLoopRampRate(0.125);
		driveLeftFrontSpark.setSmartCurrentLimit(60);


		driveLeftRearSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_REAR_ID, MotorType.kBrushless);
		driveLeftRearSpark.restoreFactoryDefaults();
    driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
    driveLeftFrontSpark.setInverted(false);
		driveLeftRearSpark.setOpenLoopRampRate(0.125);
		driveLeftRearSpark.setSmartCurrentLimit(60);

		driveRightFrontSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
		driveRightFrontSpark.restoreFactoryDefaults();
    driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
    driveLeftFrontSpark.setInverted(false);
		driveRightFrontSpark.setOpenLoopRampRate(0.125);
		driveRightFrontSpark.setSmartCurrentLimit(60);

		driveRightRearSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_REAR_ID, MotorType.kBrushless);
		driveRightRearSpark.restoreFactoryDefaults();
    driveRightRearSpark.setIdleMode(IdleMode.kBrake);
    driveLeftFrontSpark.setInverted(false);
		driveRightRearSpark.setOpenLoopRampRate(0.125);
    driveRightRearSpark.setSmartCurrentLimit(60);

    d = 55 / Math.tan(Math.toRadians(a1+a2)); // Calculates the distance between camera and target

    driveLeftFrontEncoder = driveLeftFrontSpark.getEncoder();
    driveLeftRearEncoder = driveLeftRearSpark.getEncoder();
    driveRightFrontEncoder = driveRightFrontSpark.getEncoder();
    driveRightRearEncoder = driveRightRearSpark.getEncoder();

    swerveAverageRPM = (Math.abs(driveRightFrontEncoder.getVelocity()) + Math.abs(driveRightRearEncoder.getVelocity()) + Math.abs(driveLeftRearEncoder.getVelocity()) + Math.abs(driveLeftFrontEncoder.getVelocity())) / 4;


    robotSpeed = Math.abs((((swerveAverageRPM / 10)*12.5663)/60)/12);

    // distanceCoefficiant = 1.66 + -.0064 * d + .0000103*(d*d); 
    distanceCoefficiant = 1.7 + -.00762 * d + .0000173*(d*d); 

    leadAngle = .805 + 3.13 * robotSpeed + -.261*(robotSpeed*robotSpeed)-2;

    double turretOffset = (-.946 + .0843*d + -.000508 * (d*d) + .000000884 * (d*d*d)-1 + leadAngle) * distanceCoefficiant; //original golden
    // double turretOffset = -2.64 + .11*d + -.000632 * (d*d) + .00000108 * (d*d*d)-1; //second test
    // double turretOffset = 10;
    System.out.println(d);

    // System.out.println("NetworkTableEntry tx.getDouble(0.0): " + x + turretOffset);

    double turretposition = turretencoder.getPosition();


    // Rightmost position divided by 90.
    /* if (d > 200){
      turretOffset = 3.5;
    } */

    double target = turretposition + x * (53.76 / 90.0) + turretOffset;


    turretPID = turretmotor.getPIDController();
    turretPID.setP(0.2);
    turretPID.setI(0.00);
    turretPID.setD(0.28);
    turretPID.setIZone(0.0);
    turretPID.setFF(0.0);
    turretPID.setOutputRange(-0.8, 0.8);

    turretPID.setReference(target, ControlType.kPosition);

    // System.out.println(leftJoyValue);

    /*
    if (turretencoder.getPosition() < 20 && turretencoder.getPosition() > -20){

      if (target > 0.25) {
        turretmotor.set(-0.1);
      }

      if (target < -0.25) {
        turretmotor.set(0.1);
      }
    }

    if (turretencoder.getPosition() > 20) {
    }
    */
  }


  public void left() {
    /**
     * Turns the turret to the left.
     */

    double pos = turretencoder.getPosition();

    // If we are above -45.33, we can turn left;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos > -42.82) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() > -0.03 == false) {
        // Start the motor.
        turretmotor.set(-0.85);
      }
    }
    else {
      // Stop the motor.
      turretmotor.set(0.0);
    }
  }

  public void right() {
    /**
     * Turns the turret to the right.
     */

    double pos = turretencoder.getPosition();

    // If we are below 53.76, we can turn right;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos < 51.25) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() < 0.03 == false) {
        // Start the motor.
        turretmotor.set(1);
      }
    }
    else {
      // Stop the motor.
      turretmotor.set(0.0);
    }
  }

  public void stop() {
    /**
     * Stop the motor entirely.
     */

    turretmotor.set(0.0);
    // System.out.println(turretencoder.getPosition());
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
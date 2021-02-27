/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

/**
 * Add your docs here.
 */
public class Thrower extends Subsystem {

  private CANSparkMax throwermotor;
  private CANSparkMax feedmotor;
  private CANSparkMax stackmotor;
  private CANEncoder throwermotorEncoder;
  private CANEncoder feedmotorEncoder;
  private CANEncoder stackmotorEncoder;
  private CANPIDController throwerPID;
  private NetworkTable table;
  private double a1;
  private double a2;
  private double h1;
  private double h2;
  private double d;
  private double power;
  private CANEncoder shooterEncoder;

  

  public Thrower() {
    feedmotor = new CANSparkMax(RobotMap.FEED_MOTOR_ID, MotorType.kBrushless);
    feedmotorEncoder = new CANEncoder(feedmotor);

    throwermotor = new CANSparkMax(RobotMap.THROWER_MOTOR_ID, MotorType.kBrushless);
    throwermotorEncoder = new CANEncoder(throwermotor);

    stackmotor = new CANSparkMax(RobotMap.STACK_MOTOR_ID, MotorType.kBrushless);    
    stackmotorEncoder = new CANEncoder(stackmotor);
  }

  public void thrown(){
    
    table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ledMode = table.getEntry("ledMode");
    // ledMode.forceSetNumber(1);
    
    a1 = 21; // Angel of camera from the horizontal in degrees
    a2 = ty.getDouble(0.0); // Angel of tower to camera found with limelight
    h1 = 27; // Height of limelight to ground in inches
    h2 = 82; // Height of tower's tape to ground in inches

    d = 55 / Math.tan(Math.toRadians(a1+a2)); // Calculates the distance between camera and target
  
    // Set motor power for shooting the turret.
    // Over time, the turret occasionally gets less accurate
    // use the power nerf to adjust for that.

    // int power_nerf = 4;
    // power = 4.06 + 0.346*d + -0.000475*(d*d) - power_nerf;
    // power = 25.9*Math.pow(Math.E, 0.00368*d);
    // power = -0.61;

    //power = -(2256 + -.923*d + .0128*(d*d)); //actual data, the rest are adjusted
    //power = -(2048 + 1.32*d + .0073*(d*d));
    //power = -3385;
    //power = -(2464 + -3.17*d + .0179*(d*d)); //lower power
    //power = -(1799 + 3.94*d + .00202*(d*d));
    power = -(2572 + -1.83*d + .0151*(d*d)) - 55; //The golden equation for consistency
    //power = -(1585 + 8.67*d + -.00987*(d*d)) - 55; //The golden equation updated, adjusted for long distance
    //power = -(2191 + 3.04*d + -.0022*(d*d)) - 55; //The golden equation even more updated, use this
    //power = -(2206 + 2.1*d + .00539*(d*d)) - 55;
  
    // power = -(((int)power) / 100.0) * 1.75;
    
    throwerPID = throwermotor.getPIDController();
    // throwerPID.setP(0.0028);
    /*throwerPID.setP(0.0032);
    throwerPID.setI(0.0);
    throwerPID.setD(1.0);
    throwerPID.setIZone(0.0);
    throwerPID.setFF(0.0);
    throwerPID.setOutputRange(-1.0, 0.0);*/
    throwerPID.setP(0.001);
    throwerPID.setI(0.0);
    throwerPID.setD(25.0);
    throwerPID.setIZone(0.0);
    throwerPID.setFF(0.0);
    throwerPID.setOutputRange(-1.0, 0.0);

    
    shooterEncoder = throwermotor.getEncoder();
    SmartDashboard.putNumber("Actual Ball Shooter RPM", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Robot Angle (degrees)", RobotMap.navX.getAngle());
    //System.out.println(RobotMap.navX.getFusedHeading());

    // if(d > 100) {
    //   throwerPID.setReference(-9000.0, ControlType.kVelocity);
    // }
    // else {

    throwerPID.setReference(power, ControlType.kVelocity);
    // }

    //System.out.println(d);
    SmartDashboard.putNumber("distance1", d);
    SmartDashboard.putNumber("RPM1", power);
    // throwermotor.set(power);
    
  }
/*
  public double getPercentFromRPM(double wantedRpm) {
    double actualRpm = throwermotorEncoder.getVelocity();
    double rpmDifference = wantedRpm - actualRpm;
    double powerOffset = 0.1;
    //if rpmDifference
    if (rpmDifference < 40) {
      power = power - powerOffset;
    }
    else if (rpmDifference > 40) {
      power = power + powerOffset;
    }

    if (power > 0.0) {
      power = 0.0;
    }
    else if (power < -1.0) {
      power = -1.0;
    }

    return power;
  }
*/
  public void loadshooter() {
    feedmotor.set(0.8);
  }

  public void loadstack() {
    stackmotor.set(1.0);
  }

  public void Hail() {
  throwermotor.set(-0.9);
  }

  public void stop() {
    throwermotor.set(0);
    feedmotor.set(0);
    stackmotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
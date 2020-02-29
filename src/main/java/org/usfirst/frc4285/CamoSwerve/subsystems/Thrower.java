/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
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
  private NetworkTable table;
  private double a1;
  private double a2;
  private double h1;
  private double h2;
  private double d;
  private double power;

  public void thrown(){
    throwermotor = new CANSparkMax(RobotMap.THROWER_MOTOR_ID, MotorType.kBrushless);
    
    table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry ty = table.getEntry("ty");
    a1 = 12; // Angel of camera from the horizontal in degrees
    a2 = ty.getDouble(0.0); // Angel of tower to camera found with limelight
    h1 = 27; // Height of limelight to ground in inches
    h2 = 82; // Height of tower's tape to ground in inches

    d = 55 / Math.tan(Math.toRadians(a1+a2)); // Calculates the distance between camera and target
    
    power = 4.06 + 0.346*d + -0.000475*(d*d);
    // power = 25.9*Math.pow(Math.E, 0.00368*d);
    throwermotor.set(-(int)power);

    System.out.println(-(int)power);
  }

  public void loadshooter() {
      feedmotor = new CANSparkMax(RobotMap.FEED_MOTOR_ID, MotorType.kBrushless);
      feedmotor.set(0.8);
    }

  public void loadstack() {
    
    stackmotor = new CANSparkMax(RobotMap.STACK_MOTOR_ID, MotorType.kBrushless);

    stackmotor.set(1.0);
  }

  public void Hail() {
    throwermotor = new CANSparkMax(RobotMap.THROWER_MOTOR_ID, MotorType.kBrushless);

    throwermotor.set(-0.78);
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
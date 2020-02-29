/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.lang.Math;

/**
 * Add your docs here.
 */
public class Thrower extends Subsystem {

  private CANSparkMax throwermotor;
  private CANSparkMax feedmotor;
  private CANSparkMax stackmotor;
  private Timer timer;
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
    NetworkTableEntry ty = table.getEntry("ty");
    a1 = 12;
    a2 = ty.getDouble(0.0);
    h1 = 27;
    h2 = 98;

    d = ((h2-h1) / Math.tan(a1+a2)) - 50;
    
    // power = -236 + 2.7*d + -0.00623*(d*d);
    power = 25.9*Math.pow(Math.E, 0.00368*d);
  
    throwermotor.set(power);
    System.out.println(power);
  }

  public void loadshooter() {
      feedmotor = new CANSparkMax(RobotMap.FEED_MOTOR_ID, MotorType.kBrushless);
      
      feedmotor.set(0.8);

      //Timer.delay(1.35);
    }

  public void loadstack(){
    
    stackmotor = new CANSparkMax(RobotMap.STACK_MOTOR_ID, MotorType.kBrushless);

    stackmotor.set(1.0);

    //Timer.delay(0.5);
  }

  public void stop(){
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
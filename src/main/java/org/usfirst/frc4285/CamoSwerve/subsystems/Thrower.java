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
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class Thrower extends Subsystem {

  private CANSparkMax throwermotor;
  private CANSparkMax feedmotor;
  private CANSparkMax stackmotor;

  public void thrown(){
    throwermotor = new CANSparkMax(RobotMap.THROWER_MOTOR_ID, MotorType.kBrushless);
  
    throwermotor.set(-0.60);
  }

  public void loadshooter() {
      feedmotor = new CANSparkMax(RobotMap.FEED_MOTOR_ID, MotorType.kBrushless);

      Timer.delay(1.35);
      feedmotor.set(0.8);
    }

  public void loadstack(){
    stackmotor = new CANSparkMax(RobotMap.STACK_MOTOR_ID, MotorType.kBrushless);

    Timer.delay(0.5);
    stackmotor.set(1.0);
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
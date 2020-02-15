/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax liftmotor;

  public void lift (boolean m){
    liftmotor = new CANSparkMax(RobotMap.LIFT_MOTOR_ID, MotorType.kBrushless);

    liftmotor.set(0.5);
  }

  public void liftstop (boolean m){
    liftmotor = new CANSparkMax(RobotMap.LIFT_MOTOR_ID, MotorType.kBrushless);

    liftmotor.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

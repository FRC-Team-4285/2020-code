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
public class Thrower extends Subsystem {

  private CANSparkMax throwermotor1;
  private CANSparkMax throwermotor2;
  
  public void thrown(double speed){
    throwermotor1 = new CANSparkMax(RobotMap.THROWER_MOTOR_1_ID, MotorType.kBrushless);

    throwermotor1.set(speed);
  }

  public void loadshooter(double speed){
      throwermotor2 = new CANSparkMax(RobotMap.THROWER_MOTOR_2_ID, MotorType.kBrushless);

      if(speed > .25){
        throwermotor2.set(.5);
      }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

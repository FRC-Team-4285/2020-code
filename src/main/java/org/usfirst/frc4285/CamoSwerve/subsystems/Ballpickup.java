/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

/**
 * Add your docs here.
 */
public class Ballpickup extends Subsystem {
  private TalonSRX ballpickupmotor;

  public void pickup (boolean w){
    ballpickupmotor = new TalonSRX(RobotMap.Ball_Pickup_ID);

    ballpickupmotor.set(ControlMode.PercentOutput, 0.2);
  }

  public void pickstop (boolean w){
    ballpickupmotor = new TalonSRX(RobotMap.Ball_Pickup_ID);

    ballpickupmotor.set(ControlMode.PercentOutput, 0.0);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

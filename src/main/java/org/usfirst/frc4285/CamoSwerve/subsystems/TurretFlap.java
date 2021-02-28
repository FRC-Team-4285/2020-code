/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.usfirst.frc4285.CamoSwerve.RobotMap;

public class TurretFlap extends Subsystem {
/*
 * Turret Flap Subsystem
 *
 * This system is responsible for adjusting the turret flap
 * between high goal and low goal targetting.
 */

  private final VictorSPX flapMotor;

  public TurretFlap() {
    /*
     * Constructor method.
     */

    flapMotor = new VictorSPX(RobotMap.TURRET_FLAP_ID);
  }

  public void raise() {
    /*
     * Engage spin motors and detect colors while spinning.
     */
    flapMotor.set(ControlMode.PercentOutput, 1.0);
  }
  
  public void lower() {
    /*
     * Engage spin motors and detect colors while spinning.
     */
    flapMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void stop() {
    /*
     * Stop spin motor.
     */

    flapMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

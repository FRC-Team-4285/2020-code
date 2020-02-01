/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

/**
 * Add your docs here.
 */
public class Turret extends Subsystem {

  private CANSparkMax turretmotor;
  private CANEncoder turretencoder;
  private CANPIDController turretPID;

  public void follow(){
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);
    turretPID = turretmotor.getPIDController();

    turretPID.setP(0.1);
    turretPID.setI(0.0);
    turretPID.setD(0.0);
    turretPID.setIZone(0.0);
    turretPID.setFF(0.0);
    turretPID.setOutputRange(-.1, .1);
    
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

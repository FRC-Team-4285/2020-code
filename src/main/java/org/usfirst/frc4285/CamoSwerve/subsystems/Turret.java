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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Turret extends Subsystem {

  private CANSparkMax turretmotor;
  private CANPIDController turretPID;
  private CANEncoder turretencoder;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public void following() {
    System.out.println("Hello");
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);
    turretPID = turretmotor.getPIDController();

    NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    // Read values periodically
    double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    double turretposition = turretencoder.getPosition();
    double target = turretposition - x*.2284338889;

    turretPID.setP(0.1);
    turretPID.setI(0.0);
    turretPID.setD(0.0);
    turretPID.setIZone(0.0);
    turretPID.setFF(0.0);
    turretPID.setOutputRange(-0.2, 0.2);

    turretPID.setReference(target, ControlType.kPosition);

    System.out.println(tx);

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
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);    
    turretencoder = new CANEncoder(turretmotor);

    turretmotor.set(-0.85);
    System.out.println(turretencoder.getPosition());
  }

  public void right() {
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);

    turretmotor.set(0.85);
    System.out.println(turretencoder.getPosition());
  }

  public void stop() {
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);

    turretmotor.set(0.0);
    System.out.println(turretencoder.getPosition());
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
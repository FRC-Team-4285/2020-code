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
  private NetworkTable table;

  public Turret() {
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void following() {
    NetworkTableEntry tx = table.getEntry("tx");

    // Read values periodically
    double x = tx.getDouble(0.0);
    System.out.println("NetworkTableEntry tx.getDouble(0.0): " + x);

    double turretposition = turretencoder.getPosition();

    // Rightmost position divided by 90.
    double target = turretposition + x * (53.76 / 90.0);

    turretPID = turretmotor.getPIDController();
    turretPID.setP(0.1);
    turretPID.setI(0.0);
    turretPID.setD(0.0);
    turretPID.setIZone(0.0);
    turretPID.setFF(0.0);
    turretPID.setOutputRange(-0.1, 0.1);

    turretPID.setReference(target, ControlType.kPosition);

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
    /**
     * Turns the turret to the left.
     */

    double pos = turretencoder.getPosition();
    System.out.println(pos);

    // If we are above -45.33, we can turn left;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos > -42.82) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() > 0.00 == false) {
        // Start the motor.
        turretmotor.set(-0.35);
      }
    }
    else {
      // Stop the motor.
      turretmotor.set(0.0);
    }
  }

  public void right() {
    /**
     * Turns the turret to the right.
     */

    double pos = turretencoder.getPosition();
    System.out.println(pos);
    // If we are below 53.76, we can turn right;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos < 51.25) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() < 0.0 == false) {
        // Start the motor.
        turretmotor.set(0.35);
      }
    }
    else {
      // Stop the motor.
      turretmotor.set(0.0);
    }
  }

  public void stop() {
    /**
     * Stop the motor entirely.
     */

    turretmotor.set(0.0);
    // System.out.println(turretencoder.getPosition());
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
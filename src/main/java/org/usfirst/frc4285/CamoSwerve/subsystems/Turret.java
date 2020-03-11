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
  private double a1;
  private double a2;
  private double h1;
  private double h2;
  private double d;
  private double power;

  public Turret() {
    turretmotor = new CANSparkMax(RobotMap.TURRET_ID, MotorType.kBrushless);
    turretencoder = new CANEncoder(turretmotor);
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void following() {
    NetworkTableEntry tx = table.getEntry("tx");

    // Read values periodically
    double x = tx.getDouble(0.0);

    final NetworkTableEntry ty = table.getEntry("ty");
    a1 = 21; // Angel of camera from the horizontal in degrees
    a2 = ty.getDouble(0.0); // Angel of tower to camera found with limelight
    h1 = 27; // Height of limelight to ground in inches
    h2 = 82; // Height of tower's tape to ground in inches

    d = 55 / Math.tan(Math.toRadians(a1+a2)); // Calculates the distance between camera and target

    //double turretOffset = 125 + -2.13*d + .0121 * (d*d) + -.0000223 * (d*d*d);
    double turretOffset = -.946 + .0843*d + -.000508 * (d*d) + .000000884 * (d*d*d)-1;

    System.out.println("NetworkTableEntry tx.getDouble(0.0): " + x + turretOffset);

    double turretposition = turretencoder.getPosition();

    // Rightmost position divided by 90.
    /*if (d > 200){
      turretOffset = 3.5;
    }*/
    double target = turretposition + x * (53.76 / 90.0) + turretOffset;



    turretPID = turretmotor.getPIDController();
    turretPID.setP(0.2);
    turretPID.setI(0.00);
    turretPID.setD(0.28);
    turretPID.setIZone(0.0);
    turretPID.setFF(0.0);
    turretPID.setOutputRange(-0.8, 0.8);

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

    // If we are above -45.33, we can turn left;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos > -42.82) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() > -0.03 == false) {
        // Start the motor.
        turretmotor.set(-0.85);
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

    // If we are below 53.76, we can turn right;
    // otherwise, we cannot continue because of
    // clearance issues on the robot.
    if (pos < 51.25) {
      // If the motor is already engaged, do not
      // tell it to start again.
      if (turretmotor.get() < 0.03 == false) {
        // Start the motor.
        turretmotor.set(0.85);
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
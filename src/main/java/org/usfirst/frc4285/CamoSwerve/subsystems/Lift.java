/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private CANSparkMax liftmotor;
  private CANEncoder liftencoder;
  private CANPIDController liftPID;

  public Lift() {
    liftmotor = new CANSparkMax(RobotMap.LIFT_MOTOR_ID, MotorType.kBrushless);
    liftencoder = new CANEncoder(liftmotor);
  }

  public void raise (){
    liftPID = liftmotor.getPIDController();
    liftPID.setP(0.1);
    liftPID.setI(0.0);
    liftPID.setD(0.0);
    liftPID.setIZone(0.0);
    liftPID.setFF(0.0);
    liftPID.setOutputRange(-0.2, 0.2);

    // liftPID.setReference(0.0, ControlType.kPosition);

    System.out.println("Elevator position:" + liftencoder.getPosition());

    liftmotor.set(-0.2);
  }

  public void lower() {
    liftPID = liftmotor.getPIDController();
    liftPID.setP(0.1);
    liftPID.setI(0.0);
    liftPID.setD(0.0);
    liftPID.setIZone(0.0);
    liftPID.setFF(0.0);
    liftPID.setOutputRange(-0.2, 0.2);

    // liftPID.setReference(0.0, ControlType.kPosition);

    // System.out.println("Elevator position:" + liftencoder.getPosition());
  
    liftmotor.set(0.2);
  }

  public void moveToColorWheel() {
    liftPID = liftmotor.getPIDController();
    liftPID.setP(0.1);
    liftPID.setI(0.0);
    liftPID.setD(0.0);
    liftPID.setIZone(0.0);
    liftPID.setFF(0.0);
    liftPID.setOutputRange(-0.2, 0.2);
  
    liftPID.setReference(-40.0, ControlType.kPosition);

    System.out.println("Elevator position:" + liftencoder.getPosition());
  }

  public void moveToStartingConfig() {
    liftPID = liftmotor.getPIDController();
    liftPID.setP(0.1);
    liftPID.setI(0.0);
    liftPID.setD(0.0);
    liftPID.setIZone(0.0);
    liftPID.setFF(0.0);
    liftPID.setOutputRange(-0.2, 0.2);
  
    liftPID.setReference(0.0, ControlType.kPosition);

    System.out.println("Elevator position:" + liftencoder.getPosition());
  }

  /*public void moveToStartingConfig() {
    liftPID = liftmotor.getPIDController();
    liftPID.setP(0.1);
    liftPID.setI(0.0);
    liftPID.setD(0.0);
    liftPID.setIZone(0.0);
    liftPID.setFF(0.0);
    liftPID.setOutputRange(-0.2, 0.2);
  
    liftPID.setReference(0.0, ControlType.kPosition);

    System.out.println("Elevator position:" + liftencoder.getPosition());
  }*/

  public void liftstop (){
    liftmotor.set(0.0);
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

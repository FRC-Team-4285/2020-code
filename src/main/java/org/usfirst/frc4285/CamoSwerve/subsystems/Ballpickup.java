/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4285.CamoSwerve.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

/**
 * Add your docs here.
 */
public class Ballpickup extends Subsystem {
  private CANSparkMax ballpickupmotor;
  private CANSparkMax pickupflipmotor;
  private CANPIDController pickupflipPID;
  private CANEncoder pickupflipencoder; 

  public void ballin (){
    pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);
    pickupflipencoder = new CANEncoder(pickupflipmotor);
    pickupflipPID = pickupflipmotor.getPIDController();

    pickupflipPID.setP(0.1);
    pickupflipPID.setI(0.0);
    pickupflipPID.setD(0.0);
    pickupflipPID.setIZone(0.0);
    pickupflipPID.setFF(0.0);
    pickupflipPID.setOutputRange(-.2, .2);

    //pickupflipPID.setReference(10, ControlType.kPosition);
    //pickupflipmotor.set(0.1);
    //System.out.println(pickupflipencoder.getPosition());

       ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);

    ballpickupmotor.set(0.8);
  }
   

  public void ballput (){
      pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);
    pickupflipencoder = new CANEncoder(pickupflipmotor);
    pickupflipPID = pickupflipmotor.getPIDController();

    pickupflipPID.setP(0.1);
    pickupflipPID.setI(0.0);
    pickupflipPID.setD(0.0);
    pickupflipPID.setIZone(0.0);
    pickupflipPID.setFF(0.0);
    pickupflipPID.setOutputRange(-.2, .2);

    //pickupflipPID.setReference(0, ControlType.kPosition);
    //pickupflipmotor.set(-0.1);
    //System.out.println(pickupflipencoder.getPosition());

    ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);

    ballpickupmotor.set(0.8);
  }
  public void ballrun (){
    ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);

    ballpickupmotor.set(0.8);



  }


  public void stop (){
    ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);
    pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);

    ballpickupmotor.set(0.0);
    pickupflipmotor.set(0.0);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
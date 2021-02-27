package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4285.CamoSwerve.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;


public class Ballpickup extends Subsystem {
 /*
  * Ball Pickup Subsystem
  *
  * This system is responsible for picking up power cells from the
  * ground in the game field.
  */

  private CANSparkMax ballpickupmotor;
  private CANSparkMax pickupflipmotor;
  private CANPIDController pickupflipPID;
  private CANEncoder pickupflipencoder;

  public void ballin (){
    /* 
     * Engage motors that retracts pickup system back into starting configuration.
     */

    pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);
    pickupflipencoder = new CANEncoder(pickupflipmotor);
    pickupflipPID = pickupflipmotor.getPIDController();

    pickupflipPID.setP(0.1);
    pickupflipPID.setI(0.0);
    pickupflipPID.setD(0.0);
    pickupflipPID.setIZone(0.0);
    pickupflipPID.setFF(0.0);
    pickupflipPID.setOutputRange(-.35, .35);

    pickupflipPID.setReference(52, ControlType.kPosition);
    
    // pickupflipmotor.set(0.1);
    // System.out.println(pickupflipencoder.getPositvion());
  }

  public void ballput (){
    /* 
     * Engage motors that extends pickup system so we can pick up power cells.
     */

    pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);
    pickupflipencoder = new CANEncoder(pickupflipmotor);
    pickupflipPID = pickupflipmotor.getPIDController();

    pickupflipPID.setP(0.1);
    pickupflipPID.setI(0.0);
    pickupflipPID.setD(0.0);
    pickupflipPID.setIZone(0.0);
    pickupflipPID.setFF(0.0);
    pickupflipPID.setOutputRange(-.35, .35);

    pickupflipPID.setReference(0, ControlType.kPosition);

    // pickupflipmotor.set(-0.1);
    // System.out.println(pickupflipencoder.getPosition());
  }

  public void ballrun () {
    /* 
     * Engage the feeder motor to begin picking up a ball.
     */

    ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);
    ballpickupmotor.set(.75);
  }

  public void stop() {
    /*
     * Stops the intake motors.
     */

    ballpickupmotor = new CANSparkMax(RobotMap.BALL_PICKUP_MOTOR_ID, MotorType.kBrushless);
    pickupflipmotor = new CANSparkMax(RobotMap.Pickup_Flip_ID, MotorType.kBrushless);

    ballpickupmotor.set(0.0);
    pickupflipmotor.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
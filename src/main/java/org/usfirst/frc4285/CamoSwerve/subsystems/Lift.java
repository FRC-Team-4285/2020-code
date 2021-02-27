package org.usfirst.frc4285.CamoSwerve.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.usfirst.frc4285.CamoSwerve.RobotMap;


public class Lift extends Subsystem {
    /*
     * Lift Subsystem
     *
     * This class implements the elevator/climb system. Code is designed
     * for use with our 2020 robot. This is what lifts the robot up for the
     * final challenge where robots must hang from a bar in the center of the
     * field.
     */

    private CANSparkMax liftmotor;
    private CANEncoder liftencoder;
    private CANPIDController liftPID;

    public Lift() {
      /*
       * Construct motors.
       */

      liftmotor = new CANSparkMax(RobotMap.LIFT_MOTOR_ID, MotorType.kBrushless);
      liftencoder = new CANEncoder(liftmotor);
    }

    public void raise () {
      /*
       * Raise elevator.
       */

      liftPID = liftmotor.getPIDController();
      liftPID.setP(0.1);
      liftPID.setI(0.0);
      liftPID.setD(0.0);
      liftPID.setIZone(0.0);
      liftPID.setFF(0.0);
      liftPID.setOutputRange(-0.6, 0.6);

      // liftPID.setReference(-483, ControlType.kPosition);

      System.out.println("Elevator position:" + liftencoder.getPosition());

      liftmotor.set(-0.8);
    }

    public void lower() {
      /*
       * Lower elevator.
       */

      liftPID = liftmotor.getPIDController();
      liftPID.setP(0.1);
      liftPID.setI(0.0);
      liftPID.setD(0.0);
      liftPID.setIZone(0.0);
      liftPID.setFF(0.0);
      liftPID.setOutputRange(-0.2, 0.2);
      //liftPID.setReference(-263, ControlType.kPosition);

      System.out.println("Elevator position:" + liftencoder.getPosition());
    
      liftmotor.set(0.6);
    }

    public void moveToColorWheel() {
      /*
       * Spin color wheel motor, which spins the color wheel.
       */

      liftPID = liftmotor.getPIDController();
      liftPID.setP(0.1);
      liftPID.setI(0.0);
      liftPID.setD(0.0);
      liftPID.setIZone(0.0);
      liftPID.setFF(0.0);
      liftPID.setOutputRange(-0.2, 0.2);
      liftPID.setReference(-43.0, ControlType.kPosition);

      System.out.println("Elevator position:" + liftencoder.getPosition());
    }

    public void moveToStartingConfig() {
      /*
       * Lower elevator to position to starting configuration.
       */

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

    public void liftstop () {
      liftmotor.set(0.0);
    }
    

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}

// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4285.CamoSwerve;

import org.usfirst.frc4285.CamoSwerve.commands.*;
import edu.wpi.first.wpilibj.buttons.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;


public class OI {
    /*
     * This class contians all I/O related methods,
     * such as the initialization of Joysticks and
     * Controllers.
     */

    public Joystick leftJoy;
    public Joystick rightJoy;
    public XboxController controller;
    public Button btnThrow;
    public Button btnTurretTurnRight;
    public Button btnTurretTurnLeft;
    public Button btnTurretTracking;
    public Button btnBallIntakePickup;
    public Button btnBallIntakeUp;
    public Button btnBallIntakeDrop;
    public Button btnLowBall;
    public Button btnHailMary;
    public Button btnLift;
    public Button btnLower;
    public Button btnLiftToColorWheel;
    public Button btnLiftToStartingPosition;
    public Button btnSpinColorWheel;
    public Timer time;

    public  OI() {
        //////////////////////////////////
        ///     Controller Mapping     ///
        //////////////////////////////////

        leftJoy = new Joystick(RobotMap.LEFT_JOYSTICK);
        rightJoy = new Joystick(RobotMap.RIGHT_JOYSTICK);
        controller = new XboxController(RobotMap.CONTROLLER);
    
        ///////////////////////////////
        ///      Button Mapping     ///
        ///////////////////////////////

        // Turret Shooting
        btnThrow = new JoystickButton(rightJoy, 1);
        btnThrow.whenPressed(new Throwing());

        // Turret Manual Right Turn
        btnTurretTurnRight = new JoystickButton(rightJoy, 5);
        btnTurretTurnRight.whenPressed(new TurretRight());

        // Turret Manual Left Turn
        btnTurretTurnLeft = new JoystickButton(rightJoy, 6);
        btnTurretTurnLeft.whenPressed(new TurretLeft());

        // Turret Auto-Tracking Initializer
        btnTurretTracking = new JoystickButton(rightJoy, 2);
        btnTurretTracking.whenPressed(new Follow());

        // Ball Intake Pickup
        btnBallIntakeUp = new JoystickButton(leftJoy, 1);
        btnBallIntakeUp.whenPressed(new BallIntakeUp());

        // Ball Intake Dropping
        btnBallIntakeDrop = new JoystickButton(leftJoy, 4);
        btnBallIntakeDrop.whenPressed(new BallIntakeDrop());

        // Shoots ball in low goal
        btnLowBall = new JoystickButton(leftJoy, 3);
        // btnLowBall.whenPressed(new );

        // End game shot
        btnHailMary = new JoystickButton(leftJoy, 5);
        btnHailMary.whenPressed(new HailMary());

        /*
        // Lifts elevator
        btnLift = new JoystickButton(rightJoy, 4);
        btnLift.whenPressed(new Lifting());

        // Lowers elevator
        btnLower = new JoystickButton(rightJoy, 3);
        btnLower.whenPressed(new Lowering());

        // Lifts Elevator to Color Wheel
        btnLiftToStartingPosition = new JoystickButton(rightJoy, 10);
        btnLiftToStartingPosition.whenPressed(new LiftPosStartingConfig());

        // Spins Color Wheel
        btnLiftToColorWheel = new JoystickButton(rightJoy, 11);
        btnLiftToColorWheel.whenPressed(new LiftPosColorWheel());

        // Spins Color Wheel
        btnSpinColorWheel = new JoystickButton(rightJoy, 9);
        btnSpinColorWheel.whenPressed(new SpinColorWheel());
        */    
    }

    public boolean getButtonTurretFollow() {
        /*
         * DEPRECATED: Do not write new code using this method.
         *
         * Returns the activity status of the button
         * mapped to initialize the turret following AI.
         */

        return rightJoy.getRawButtonReleased(2); 
    }

    public boolean getButtonLeftTurret() {
        /*
         * Returns the activity status of the button
         * mapped to turning turret to the left.
         */

        return rightJoy.getRawButtonReleased(6);
    }

    public boolean getButtonRightTurret() {
        /*
         * Returns the activity status of the button
         * mapped to turning turret to the right.
         */

        return rightJoy.getRawButtonReleased(5);
    }

    public boolean getButtonTurretShoot() {
        /*
         * Returns the activity status of the button
         * mapped to shooting the turret.
         */

        return rightJoy.getRawButtonReleased(1);
    }

    public boolean getButtonBallPickUp() {
        /*
         * Returns the activity status of the button
         * mapped to enable the ball intake to pick
         * up a ball.
         */

        return leftJoy.getRawButtonReleased(1);
    }

    public boolean getButtonBallDrop() {
        /*
         * Returns the activity status of the button
         * mapped to enable the ball intake to drop
         * the ball into the feeder.
         */

         return leftJoy.getRawButtonReleased(4);
    }

    public boolean getButtonLowerFlap() {
        /*
         * Returns the activity status of the button
         * mapped to raise turret flap to target high
         * goal.
         */

        return leftJoy.getRawButtonReleased(3);
    }

    public boolean getButtonRaiseFlap() {
        /*
         * Returns the activity status of the button
         * mapped to raise turret flap to target high
         * goal.
         */

        return leftJoy.getRawButtonReleased(4);
    }

    public boolean getLeftJoyButton(int buttonNumber) {
        /*
         * Returns the activity status of the button number
         * associated with 'buttonNumber'.
         */

        return leftJoy.getRawButton(buttonNumber);
    }

    public boolean getRightJoyButton(int buttonNumber) {
        /*
         * Returns the activity status of the button number
         * associated with 'buttonNumber'.
         */

        return rightJoy.getRawButton(buttonNumber);
    }

    public boolean getButtonHailMary() {
        /*
         * Returns the activity status of the button
         * mapped to enable the end game shot at 75%
         */

        return leftJoy.getRawButtonReleased(5);
    }
    
    public boolean getButtonLiftRaise() {
        
         // Returns the activity status of the button
         // mapped to enable the elevator to go up.
         
        return rightJoy.getRawButtonReleased(4);
    }

    public boolean getButtonLiftLower() {
        
         // Returns the activity status of the button
         // mapped to enable the elevator to go down.
         
        return rightJoy.getRawButtonReleased(3);
    }
    
    public double getRightTrigger() {
        /*
         * Returns the activity status of the right
         * trigger.
         */
        return controller.getRawAxis(3);
    }
    
    public boolean getButtonSpinColorWheel() {
        /*
         * Returns the activity status of the button
         * mapped to enable the elevator to go down.
         */
        return rightJoy.getRawButtonReleased(9);
    }

    public boolean getButtonLiftToColorWheel() {
        /*
         * Returns the activity status of the button
         * mapped to enable the elevator to go down.
         */
        return rightJoy.getRawButtonReleased(11);
    }

    public boolean getButtonLiftToStartingPosition() {
        /*
         * Returns the activity status of the button
         * mapped to enable the elevator to go down.
         */
        return rightJoy.getRawButtonReleased(12);
    }

    public boolean getcontrollerAbuttonpress () {
        /*
         * Returns the activity status of the right
         * trigger.
         */
        return controller.getAButtonPressed();
    }

    public boolean getcontrollerAbuttonrelease () {
        return controller.getAButtonReleased();
    }

    public boolean getcontrollerBbuttonpress (){
        return controller.getBButtonPressed();
    }

    public boolean getcontrollerBbuttonrelease (){
        return controller.getBButtonReleased();
    }
}
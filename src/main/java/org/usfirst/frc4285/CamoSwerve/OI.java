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
    public Button btnBallIntakeDrop;
    public Button button7;

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
        // btnThrow = new JoystickButton(leftJoy, 3);
        // btnThrow.whenPressed(new Throwing());

        // Turret Manual Right Turn
        btnTurretTurnRight = new JoystickButton(rightJoy, 5);
        btnTurretTurnRight.whenPressed(new TurretRight());

        // Turret Manual Left Turn
        btnTurretTurnLeft = new JoystickButton(rightJoy, 4);
        btnTurretTurnLeft.whenPressed(new TurretLeft());

        // DEBUG: Turret Auto-Tracking Initializer
        btnTurretTracking = new JoystickButton(rightJoy, 3);
        btnTurretTracking.whenPressed(new Follow());

        // Ball Intake Pickup/Dropping
        // btnBallIntakeDrop = new JoystickButton(leftJoy, 5);
        // btnBallIntakeDrop.whenPressed(new Pickupball());


        //////////////////////////////////////
        ///     PENDING IMPLEMENTATION     ///
        //////////////////////////////////////

        // btnBallIntakePickup = new JoystickButton(leftJoy, 4);
        // btnBallIntakePickup.whenPressed(new Pickupballput());

        // DEBUG: Ball Intake Initializer
        // btnRunBallIntake = new JoystickButton(leftJoy, 2);
        // btnRunBallIntake.whenPressed(new BallIntake());
    
    }

    public boolean getButtonTurretFollow() {
        /*
         * DEPRECATED: Do not write new code using this method.
         *
         * Returns the activity status of the button
         * mapped to initialize the turret following AI.
         */

        return rightJoy.getRawButtonReleased(3); 
    }

    public boolean getButtonLeftTurret() {
        /*
         * Returns the activity status of the button
         * mapped to turning turret to the left.
         */

        return rightJoy.getRawButtonReleased(4);
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

        return leftJoy.getRawButtonReleased(3);
    }

    public boolean getButtonBallPickup() {
        /*
         * Returns the activity status of the button
         * mapped to enable the ball intake to pick
         * up a ball.
         */

        return leftJoy.getRawButtonReleased(5);
    }

    public boolean getLeftJoyButton (int buttonNumber) {
        /*
         * Returns the activity status of the button number
         * associated with 'buttonNumber'.
         */

        return leftJoy.getRawButton(buttonNumber);
    }

    public double getRightTrigger () {
        /*
         * Returns the activity status of the right
         * trigger.
         */
        return controller.getRawAxis(3);
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
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.commands;

import org.usfirst.frc4285.CamoSwerve.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ThrowingAutonomous extends Command {

  public ThrowingAutonomous() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.thrower);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(6);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timeSinceInitialized() > 0) {
      Robot.thrower.thrown();
    }
    if (timeSinceInitialized() > 1) {
      Robot.thrower.thrown();
      Robot.thrower.loadstack();
    }  
    if(timeSinceInitialized() > 1.85) {
      Robot.thrower.thrown();
      Robot.thrower.loadstack();
      Robot.thrower.loadshooter();
 } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.thrower.stop();
    Robot.drive.swerveDrive(0.0, 0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
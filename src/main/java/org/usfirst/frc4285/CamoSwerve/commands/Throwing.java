/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4285.CamoSwerve.commands;

import org.usfirst.frc4285.CamoSwerve.Robot;

import edu.wpi.first.wpilibj.command.Command;


public class Throwing extends Command {

  public Throwing() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.thrower);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timeSinceInitialized() > 0) {
      Robot.thrower.thrown();
    }
    if (timeSinceInitialized() > 0.5) {
      Robot.thrower.thrown();
      Robot.thrower.loadshooter();
    }  
    if(timeSinceInitialized() > 2.5) {
      Robot.thrower.thrown();
      Robot.thrower.loadstack();
      Robot.thrower.loadshooter();
 } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.getButtonTurretShoot();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.thrower.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
package org.usfirst.frc4285.CamoSwerve.commands;

import org.usfirst.frc4285.CamoSwerve.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class HailMary extends Command {
  public HailMary() {
    requires(Robot.thrower);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timeSinceInitialized() > 0) {
      Robot.thrower.Hail();
    }
    if (timeSinceInitialized() > 0.5) {
      Robot.thrower.Hail();
      Robot.thrower.loadstack();
    }  
    if(timeSinceInitialized() > 1.35) {
      Robot.thrower.Hail();
      Robot.thrower.loadstack();
      Robot.thrower.loadshooter();
 } 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.getButtonHailMary();
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

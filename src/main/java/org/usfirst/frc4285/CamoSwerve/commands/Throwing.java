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
    // Engage initial motors to feed ball in to the shooter.
    if (timeSinceInitialized() > 0) {
      Robot.thrower.thrown();
    }

    // Start moving the ball up inside the shooter near the turret.
    if (timeSinceInitialized() > 0.5) {
      Robot.thrower.thrown();
      Robot.thrower.loadshooter();
    }  

    // Shoot the ball out of the turret.
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
package org.usfirst.frc4285.CamoSwerve.commands;

import org.usfirst.frc4285.CamoSwerve.Robot;

import edu.wpi.first.wpilibj.command.Command;


public class SpinColorWheel extends Command {

  public SpinColorWheel() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.colorwheel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.colorwheel.spin();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.getButtonSpinColorWheel();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.colorwheel.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
package org.usfirst.frc4285.CamoSwerve.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4285.CamoSwerve.Robot;
import org.usfirst.frc4285.CamoSwerve.RobotMap;


public class PointNAV extends Command {
  public PointNAV() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // todo replace with angle
    double originHeading = Robot.zeroHeading;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double strafe = 0.0;
    double forward = 0.0;
    double omega = 0.0;

    Robot.drive.swerveDrive(strafe, forward, omega);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

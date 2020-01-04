/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utils.MathUtils;

public class AccelerateStraightToTarget extends DriveStraightToTarget {
  double startingPower;
  double endingPower;
  double accelerationToTopSpeedRatio;

  double prev_throttle;

  public AccelerateStraightToTarget(double distance, double angle, double startingPower, double endingPower, double accelerationToTopSpeedRatio) {
    super(distance, angle);
    this.startingPower = startingPower;
    this.endingPower = endingPower;
    this.accelerationToTopSpeedRatio = accelerationToTopSpeedRatio;
  }

  public AccelerateStraightToTarget(double distance, double angle, double startingPower, double endingPower) {
    this(distance, angle, startingPower, endingPower, 1.0);
  }

  public AccelerateStraightToTarget(double distance, double angle, double endingPower) {
    //TODO: determine minimum power
    this(distance, angle, 0.0, endingPower, 1.0);
  }
  
  public AccelerateStraightToTarget(double distance, double angle) {
    //TODO: determine minimum power
    this(distance, angle, 0.0, 1.0, 1.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    super.initialize();

    prev_throttle = Robot.m_driveTrain.getThrottle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double accel_dist = accelerationToTopSpeedRatio * distance;
    double alpha = MathUtils.clamp01(Robot.m_driveTrain.getDistance()/accel_dist);
    double output = MathUtils.lerp(startingPower, endingPower, alpha);
    Robot.m_driveTrain.setThrottle(output);    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return super.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.setThrottle(prev_throttle);
    super.end();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.setThrottle(prev_throttle);
    super.interrupted();
  }
}

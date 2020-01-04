/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.State;

public class DriveStraightToTarget extends Command {
  protected double distance;
  protected double angle;
/**
 * Drives straight at given heading
 * @param distance The target distance in inches
 */
public DriveStraightToTarget(double distance, double angle){
  this.distance = distance;
  this.angle = angle;
  requires(Robot.m_driveTrain);
}
public DriveStraightToTarget(double distance){
  this.distance = distance;
  this.angle = Robot.m_driveTrain.getHeading();
  requires(Robot.m_driveTrain);
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.m_driveTrain.zeroGyro();
    Robot.m_driveTrain.zeroEncoders();
    Robot.m_driveTrain.setTargetDistance(distance);
    Robot.m_driveTrain.setTargetAngle(angle);
    Robot.m_driveTrain.setState(State.LOW_GEAR_CLOSED_LOOP);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_driveTrain.onTargetPosition() && Robot.m_driveTrain.onTargetAngle();
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

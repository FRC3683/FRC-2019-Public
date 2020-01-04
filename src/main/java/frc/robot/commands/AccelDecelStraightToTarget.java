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

public class AccelDecelStraightToTarget extends DriveStraightToTarget {
  double startingPower;
  double maxPower;
  double endingPower;
  double accelerationToTopSpeedRatio;
  double decelerationToTopSpeedRatio;

  double prev_throttle;


  public AccelDecelStraightToTarget(double distance, double angle, double startingPower, double maxPower, double endingPower, 
                                    double accelerationToTopSpeedRatio, double decelerationToTopSpeedRatio) {
    super(distance, angle);
    this.startingPower = startingPower;
    this.maxPower = maxPower;
    this.endingPower = endingPower;
    this.accelerationToTopSpeedRatio = accelerationToTopSpeedRatio;
    this.decelerationToTopSpeedRatio = decelerationToTopSpeedRatio;
  }

  public AccelDecelStraightToTarget(double distance, double angle, double startingPower, double maxPower, double endingPower, 
                                    double accelDecelToTopSpeedRatio){
    this(distance, angle, startingPower, maxPower, endingPower, accelDecelToTopSpeedRatio, accelDecelToTopSpeedRatio);
  }

  public AccelDecelStraightToTarget(double distance, double angle, double startingPower, double maxPower, double endingPower){
    this(distance, angle, startingPower, maxPower, endingPower, 0.33, 0.33);
  }

  public AccelDecelStraightToTarget(double distance, double angle, double startAndEndPower, double maxPower){
    this(distance, angle, startAndEndPower, maxPower, startAndEndPower);
  }
  
  public AccelDecelStraightToTarget(double distance, double angle, double startAndEndPower){
    this(distance, angle, startAndEndPower, 1.0, startAndEndPower);
  }
  
  public AccelDecelStraightToTarget(double distance, double angle){
    //TODO: find appropriate min power
    this(distance, angle, 0.0, 1.0, 0.0);
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
    double decel_dist = decelerationToTopSpeedRatio * distance;
    double zeroDist = (1.0 - decelerationToTopSpeedRatio)*distance;

    double alpha = 0.0;
    double output = 0.0;

    if(Robot.m_driveTrain.getDistance() > accel_dist){
      alpha = MathUtils.clamp01(Robot.m_driveTrain.getDistance()/accel_dist);
      output = MathUtils.lerp(startingPower, maxPower, alpha);
    } else{
      alpha = MathUtils.clamp01((Robot.m_driveTrain.getDistance() - zeroDist)/decel_dist);
      output = MathUtils.lerp(maxPower, endingPower, alpha);
    }
    
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

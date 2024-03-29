/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm.State;

public class ArmToPosition extends Command {
  
  private State target;
  
  public ArmToPosition(State target) {
    requires(Robot.m_arm);
    this.target = target;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_arm.setTargetState(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Set the robot state to the desired state?  
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_arm.getCurrentState() == target;
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

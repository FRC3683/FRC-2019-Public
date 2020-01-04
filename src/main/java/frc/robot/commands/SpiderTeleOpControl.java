/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Spider.State;

public class SpiderTeleOpControl extends Command {
  private OI oi;

  public SpiderTeleOpControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_spider);
    oi = OI.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_spider.setState(State.DISENGAGED);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (oi.getXButtonPressedDriver() && Robot.climbing){
      if (Robot.m_spider.getState() == State.DISENGAGED) {
        Robot.m_spider.setState(State.ENGAGED);
      }
      else if (Robot.m_spider.getState() == State.ENGAGED) {
        Robot.m_spider.setState(State.DISENGAGED);
      }
    }
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
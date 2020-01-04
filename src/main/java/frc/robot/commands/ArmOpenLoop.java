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
import frc.robot.subsystems.Arm.State;

public class ArmOpenLoop extends Command {

  private OI oi;
  private double output = 0.0;

  public ArmOpenLoop() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {    
     oi = OI.getInstance();
     Robot.m_arm.setTargetState(State.OPEN_LOOP);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_arm.getCurrentState() == State.OPEN_LOOP){
      output = oi.getYLeftDriver();
      System.out.print("open loop output: ");
      if(oi.getAButtonDriver()){
        Robot.m_arm.zeroEncoder();
      }
    }
    System.out.println(output);
    Robot.m_arm.setOutput(0.4 * output);
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

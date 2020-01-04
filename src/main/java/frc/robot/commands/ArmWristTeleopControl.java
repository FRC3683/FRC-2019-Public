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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand.HandState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;

public class ArmWristTeleopControl extends Command {
	private OI oi;
	private boolean shouldStagger;
	private boolean staggerArm;
  
  public ArmWristTeleopControl() {
    // Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	shouldStagger = false;
	staggerArm = false;
	Robot.climbing = false;
	oi = OI.getInstance();
    requires(Robot.m_arm);
	requires(Robot.m_wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run 
  @Override
	 /**
	  * Get a button for each of the states
	  */
  	protected void execute() {
		if(!Robot.climbing && oi.getStartButtonPressedDriver()){//Robot.m_spider.getState() == Spider.State.ENGAGED){
			Robot.climbing = true;
		}
		if(Robot.climbing){
			climb();
		}
		else if(Robot.m_arm.getCurrentState() == Arm.State.OPEN_LOOP && Robot.m_wrist.getCurrentState() == Wrist.State.OPEN_LOOP){
			openLoop();
		}
		else if(Robot.m_hand.getHandState() == HandState.BALL_MODE){
			ballGame();
		}
		else{
			discGame();
		}
		if(shouldStagger){	
			staggerMotion();
		}
	}

  	private void climb() { 
		if(oi.getBackButtonDriver()){
			Robot.m_arm.setTargetState(Arm.State.CLIMB);
			Robot.m_wrist.setTargetState(Wrist.State.CLIMB);
		}
		else{
			Robot.m_arm.setTargetState(Arm.State.CLIMB_SETUP);
			Robot.m_wrist.setTargetState(Wrist.State.CLIMB_SETUP);
		}
		if(oi.getYButtonDriver() && oi.getBButtonDriver()){
			Robot.exit_climb = true;
			Robot.climbing = false;
			Robot.m_wrist.setTargetState(Wrist.State.STOWED);
			//Robot.m_arm.setTargetState(Arm.State.STOWED);
			shouldStagger = true;
			staggerArm = true;
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

	private void openLoop(){
		Robot.m_arm.setOutput(-0.5 * oi.getYLeftOperator());
		Robot.m_wrist.setOutput(-0.5 * oi.getYRightOperator());
		if(oi.getDPadDownOperator()){
			Robot.m_arm.zeroEncoder();
		}
		if(oi.getBackButtonOperator()){
			Robot.m_arm.setTargetState(Arm.State.STOWED);
			Robot.m_wrist.setTargetState(Wrist.State.STOWED);
		}
	}

	private void ballGame() {
		if (oi.getAButtonPressedOperator()) {
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_LOW);
			Robot.m_arm.setTargetState(Arm.State.BALL_LOW);	
			shouldStagger = true;
			staggerArm = false;
		}
		else if (oi.getBButtonPressedOperator()){
			Robot.m_arm.setTargetState(Arm.State.BALL_MID);	
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_MID);
			shouldStagger = true;	
			staggerArm = false;	
        }
		else if (oi.getXButtonPressedOperator()){
			Robot.m_arm.setTargetState(Arm.State.BALL_CARGO);
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_CARGO);
			shouldStagger = true;
			staggerArm = false;
		}
		else if (oi.getYButtonPressedOperator()){
			Robot.m_arm.setTargetState(Arm.State.BALL_HIGH);
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_HIGH);
			shouldStagger = true;
			staggerArm = false;
		}
		else if(oi.getLeftTriggerOperator() >= 0.5){
			Robot.m_arm.setTargetState(Arm.State.BALL_HP);
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_HP);
			shouldStagger = true;
			staggerArm = false;
		}
		else if(oi.getRightTriggerDriver() > 0.5 && Robot.m_arm.getCurrentState() == Arm.State.STOWED){
			Robot.m_arm.setTargetState(Arm.State.BALL_FLOOR);
			//Robot.m_wrist.setTargetState(Wrist.State.BALL_FLOOR);
			shouldStagger = true;
			staggerArm = false;
		}
		else if (oi.getLeftStickButtonPressedOperator() || (oi.getRightTriggerDriver() < 0.5 && Robot.m_wrist.getTargetState() == State.BALL_FLOOR)) {
            Robot.m_wrist.setTargetState(Wrist.State.STOWED);
			//Robot.m_arm.setTargetState(Arm.State.STOWED);
			shouldStagger = true;
			staggerArm = true;
		}else if(oi.getStartButtonOperator()){
			Robot.m_arm.setTargetState(Arm.State.OPEN_LOOP);
			Robot.m_wrist.setTargetState(Wrist.State.OPEN_LOOP);
			shouldStagger = false;
			staggerArm = false;
		}
	}

	private void discGame() {
		if (oi.getAButtonPressedOperator()) {
            Robot.m_wrist.setTargetState(Wrist.State.DISC_LOW);
			//Robot.m_arm.setTargetState(Arm.State.DISC_LOW);
			shouldStagger = true;
			staggerArm = true;
		}
		else if (oi.getBButtonPressedOperator()){
            Robot.m_arm.setTargetState(Arm.State.DISC_MID);
            //Robot.m_wrist.setTargetState(Wrist.State.DISC_MID);
			shouldStagger = true;
			staggerArm = false;
		}
		else if (oi.getXButtonPressedOperator()){
			//empty button handler
		}
		else if (oi.getYButtonPressedOperator()){
            Robot.m_arm.setTargetState(Arm.State.DISC_HIGH);
            //Robot.m_wrist.setTargetState(Wrist.State.DISC_HIGH);
			shouldStagger = true;
			staggerArm = false;
		}
		else if(oi.getRightTriggerDriver() > 0.5 && Robot.m_arm.getCurrentState() == Arm.State.STOWED){
            Robot.m_arm.setTargetState(Arm.State.DISC_HP);
            //Robot.m_wrist.setTargetState(Wrist.State.DISC_HP);
			shouldStagger = true;
			staggerArm = false;
		}
		else if (oi.getLeftStickButtonPressedOperator() || (oi.getRightTriggerDriver() < 0.5 && Robot.m_wrist.getTargetState() == State.DISC_HP)) {
            Robot.m_wrist.setTargetState(Wrist.State.STOWED);
            //Robot.m_arm.setTargetState(Arm.State.STOWED);
			shouldStagger = true;
			staggerArm = true;
		}if(oi.getStartButtonOperator()){
			Robot.m_arm.setTargetState(Arm.State.OPEN_LOOP);
			Robot.m_wrist.setTargetState(Wrist.State.OPEN_LOOP);
			shouldStagger = false;
			staggerArm = false;
		}
	}
	
	private void staggerMotion() {
		// Motions where arms move first
		if(Robot.m_arm.staggerReady() && !staggerArm){
			switch (Robot.m_arm.getTargetState()) {
				case BALL_LOW:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_LOW);
					shouldStagger = false;
					break;
				case BALL_MID:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_MID);
					shouldStagger = false;
					break;
				case BALL_HIGH:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_HIGH);
					shouldStagger = false;
					break;
				case BALL_CARGO:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_CARGO);
					shouldStagger = false;
					break;
				case BALL_HP:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_HP);
					shouldStagger = false;
					break;
				case DISC_MID:
					Robot.m_wrist.setTargetState(Wrist.State.DISC_MID);
					shouldStagger = false;
					break;			
				case DISC_HP:
					Robot.m_wrist.setTargetState(Wrist.State.DISC_HP);
					shouldStagger = false;
					break;
				case DISC_HIGH:
					Robot.m_wrist.setTargetState(Wrist.State.DISC_HIGH);
					shouldStagger = false;
					break;
				case BALL_FLOOR:
					Robot.m_wrist.setTargetState(Wrist.State.BALL_FLOOR);
					shouldStagger = false;
					break;
			}
		}
		// Motions where wrist moves firsts
		if(Robot.m_wrist.staggerReady() && staggerArm){
			switch (Robot.m_wrist.getTargetState()) {
				case DISC_LOW:
					Robot.m_arm.setTargetState(Arm.State.DISC_LOW);
					shouldStagger = false;
					break;
				case BALL_FLOOR:
					Robot.m_arm.setTargetState(Arm.State.BALL_FLOOR);
					shouldStagger = false;
					break;
				case STOWED:
					if(Robot.m_arm.getCurrentState() == Arm.State.BALL_FLOOR){
						if(Robot.m_wrist.getAngle() >= 70){
							Robot.m_arm.setTargetState(Arm.State.STOWED);
							shouldStagger = false;
						}
					}
					else{
						Robot.m_arm.setTargetState(Arm.State.STOWED);
						shouldStagger = false;
					}
					break;
				case LOADED_STOWED:
					Robot.m_arm.setTargetState(Arm.State.LOADED_STOWED);
					shouldStagger = false;
					break;
			}
		}
	}
}

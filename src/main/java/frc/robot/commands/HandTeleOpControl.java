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
import frc.robot.subsystems.Hand.IntakeState;

public class HandTeleOpControl extends Command {
	private OI oi;

	private enum mode {
		BALL, DISC
	}
	public HandTeleOpControl() {
		oi = OI.getInstance();
		requires(Robot.m_hand);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		switch (Robot.m_hand.getHandState()) {
			case BALL_MODE:
				ballGame();
				if (oi.getLeftBumperOperator() && !oi.getRightBumperOperator() && Robot.m_arm.getCurrentState() == Arm.State.STOWED) {
					Robot.m_hand.setIntakeState(IntakeState.STOPPED);
					Robot.m_hand.setHandState(HandState.EMPTY_DISC);
				}
				break;
			case EMPTY_DISC:
				Robot.m_hand.setIntakeState(IntakeState.STOPPED);
				if (oi.getRightTriggerDriver() > 0.5) {//&& Robot.m_arm.getCurrentState() == Arm.State.DISC_HP) {
					Robot.m_hand.setHandState(HandState.PICKUP_DISC);
				} else if (oi.getRightBumperOperator() && Robot.m_arm.getCurrentState() == Arm.State.STOWED) {
					Robot.m_hand.setHandState(HandState.BALL_MODE);
				}
				break;
			case PICKUP_DISC:
				if (oi.getRightTriggerDriver() < 0.5) {
					Robot.m_hand.setHandState(HandState.HOLDING_DISC);
				}
				break;
			case HOLDING_DISC:
				if (oi.getLeftTriggerDriver() > 0.5) {
					Robot.m_hand.setHandState(HandState.JUST_SHOT);
				}
				break;
			case JUST_SHOT:
				if (oi.getLeftTriggerDriver() < 0.5) {
					Robot.m_hand.setHandState(HandState.EMPTY_DISC);
				}
				break;
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

	private void ballGame() {
		if (oi.getLeftTriggerDriver() > 0.5) {
			Robot.m_hand.setIntakeState(IntakeState.SHOOT);
		} else if (oi.getRightTriggerDriver() > 0.5) {// && Robot.m_arm.getCurrentState() == Arm.State.BALL_FLOOR) {
			Robot.m_hand.setIntakeState(IntakeState.INTAKE_FLOOR);
		} else if (oi.getRightTriggerOperator() > 0.5) {
			Robot.m_hand.setIntakeState(IntakeState.INTAKE_FLOOR);
		} else {
			Robot.m_hand.setIntakeState(IntakeState.HOLDING_BALL);
		}
	}
}

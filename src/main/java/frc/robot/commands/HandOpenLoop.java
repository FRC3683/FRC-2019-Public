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
import frc.robot.subsystems.Hand.HandState;
import frc.robot.subsystems.Hand.IntakeState;

	public class HandOpenLoop extends Command {

	private OI oi;
	private double intakeOutput = 0.0;

	public HandOpenLoop() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.m_hand);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		oi = OI.getInstance();
		Robot.m_hand.setHandState(HandState.OPEN_LOOP);
		Robot.m_hand.setIntakeState(IntakeState.OPEN_LOOP);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(Robot.m_hand.getIntakeState() == IntakeState.OPEN_LOOP){
			intakeOutput = oi.getLeftTriggerDriver() - oi.getRightTriggerDriver();
			if(oi.getAButtonDriver()){
				Robot.m_hand.openJaw();;
			}
			else{
				Robot.m_hand.closeJaw();
			}
			if(oi.getBButtonDriver()){
				Robot.m_hand.closeDisc();
			}
			else{
				Robot.m_hand.openDisc();
			}
			if(oi.getXButtonDriver()){
				Robot.m_hand.retractPunch();
			}
			else{
				Robot.m_hand.punchDisc();
			}
		}
		Robot.m_hand.setIntakeOutput(0.7 * intakeOutput);
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

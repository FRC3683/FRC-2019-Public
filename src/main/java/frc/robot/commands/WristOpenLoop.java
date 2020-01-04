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
	import frc.robot.subsystems.Wrist.State;

	public class WristOpenLoop extends Command {

	private OI oi;
	private double wristOutput = 0.0;

	public WristOpenLoop() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.m_wrist);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		oi = OI.getInstance();
		Robot.m_wrist.setTargetState(State.OPEN_LOOP);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(Robot.m_wrist.getCurrentState() == State.OPEN_LOOP){
			wristOutput = oi.getYRightDriver();
		}
		Robot.m_wrist.setOutput(0.4 * wristOutput);
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

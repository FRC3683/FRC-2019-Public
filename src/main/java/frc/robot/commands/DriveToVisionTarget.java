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
import frc.robot.utils.MathUtils;
import frc.robot.vision.Eye;

public class DriveToVisionTarget extends Command {
	private Eye eye;
	private double leftOutput;
	private double rightOutput;

	public DriveToVisionTarget() {
		eye = Eye.getInstance();
		requires(Robot.m_driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.m_driveTrain.setState(State.LOW_GEAR_OPEN_LOOP);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		leftOutput = MathUtils.calcLeftDrive(eye.getDistanceCorrection(), eye.getHeadingCorrection());
		rightOutput = MathUtils.calcRightDrive(eye.getDistanceCorrection(), eye.getHeadingCorrection());

		Robot.m_driveTrain.setLeft(leftOutput);
		Robot.m_driveTrain.setRight(rightOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return eye.targetAligned() && eye.targetCloseEnough();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.m_driveTrain.setLeft(0);
		Robot.m_driveTrain.setRight(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}

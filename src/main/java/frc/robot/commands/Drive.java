/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import org.graalvm.compiler.loop.MathUtil;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.State;
import frc.robot.subsystems.Spider;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Eye;

public class Drive extends Command {
	
	private OI oi;
	private Eye eye;
	private double Y; 
	private double X;
	private double leftOutput;
	private double rightOutput;
	private boolean visionTracking;
	private double throttle = 1.0;

	public enum DriveMode{
		ARCADE_ONE_STICK, ARCADE_TWO_STICK, TANK; 
	}
	
	DriveMode dMode;
	
	public Drive(DriveMode mode) {
		requires(Robot.m_driveTrain);
		dMode = mode;
	}
  
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		oi = OI.getInstance();
		eye = Robot.m_eye;
		Y = 0;
		X = 0;
	}
  
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(oi.getAButtonDriver()) {
			throttle = 0.50;
		} else if(oi.getLeftBumperDriver()) { //Sketchy Thing to throttle
			throttle = 0.35;	
		} else {
			throttle = 1.0;
		}
		if(Robot.m_driveTrain.getState() == State.HIGH_GEAR_OPEN_LOOP
			|| Robot.m_driveTrain.getState() == State.LOW_GEAR_OPEN_LOOP){
			switch(dMode){
				case ARCADE_ONE_STICK:
					X = MathUtils.squaredInput(oi.getXLeftDriver());
					Y = MathUtils.squaredInput(oi.getYLeftDriver());
					leftOutput = MathUtils.calcLeftDrive(Y, X);
					rightOutput= MathUtils.calcRightDrive(Y, X);
					break;
				case ARCADE_TWO_STICK:
					X = MathUtils.squaredInput(oi.getXRightDriver());
					if (Math.abs(X) > 0.15 && throttle == 1.0) throttle = 0.80;
					Y = MathUtils.squaredInput(oi.getYLeftDriver());
					leftOutput = MathUtils.calcLeftDrive(Y, X)*throttle;
					rightOutput = MathUtils.calcRightDrive(Y, X)*throttle;
					break;
				case TANK:
					X = MathUtils.squaredInput(oi.getYRightDriver());
					Y = MathUtils.squaredInput(oi.getYLeftDriver());
					leftOutput = Y;
					rightOutput= -X;
					break;  
			}
			switch (Robot.m_arm.getCurrentState()){
				case DISC_HP:
				case BALL_HP:
				case BALL_FLOOR:
				case LOADED_STOWED:
				case STOWED:
				case DISC_LOW:
				case BALL_LOW:
				case DISABLED:
					if(oi.getRightBumperDriver()){
						Robot.m_driveTrain.setState(State.HIGH_GEAR_OPEN_LOOP);
					}
					else{
						Robot.m_driveTrain.setState(State.LOW_GEAR_OPEN_LOOP);
					}
					break;
				case DISC_MID:
				case DISC_HIGH:
				case BALL_MID:
				case BALL_HIGH:
				case BALL_CARGO:
					Robot.m_driveTrain.setState(State.LOW_GEAR_OPEN_LOOP);
					break;
			}

			if (Robot.m_spider.getState() == Spider.State.ENGAGED) {
				Robot.m_driveTrain.setState(State.LOW_GEAR_OPEN_LOOP);
			}
		}
		if(oi.getAButtonDriver()){
			eye.look();
			visionAssist();
		}
		else{
			eye.close();
			oi.rumbleDriver(0.0);
			oi.rumbleOperator(0.0);
		}
		Robot.m_driveTrain.setLeft(leftOutput);
		Robot.m_driveTrain.setRight(rightOutput);
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

	public void visionAssist() {
		eye.updateTracking();
		if (eye.validTarget()){
			leftOutput = MathUtils.calcLeftDrive(Y*throttle, X + Robot.m_driveTrain.visionRotateAssist());
			rightOutput = MathUtils.calcRightDrive(Y*throttle, X + Robot.m_driveTrain.visionRotateAssist());
			if(eye.targetAligned() || Robot.m_driveTrain.onTargetAngle()){
				oi.rumbleDriver(0.5);
				oi.rumbleOperator(0.4);
				if(eye.targetCloseEnough()){
					oi.rumbleDriver(0.85);
					oi.rumbleOperator(0.85);
				}
			}
			else{
				oi.rumbleDriver(0.0);
				oi.rumbleOperator(0.0);
			}
		}
	}
}

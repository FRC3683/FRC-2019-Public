/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.utils.PID;

/**
 * Add your docs here.
 */
	public class Eye {
	private static Eye instance;

	private boolean m_EyeHasValidTarget;
	private boolean captured;
	private double targetYawError;
	private double targetSizeObserved;
	private PID yawPID;
	private PID sizePID;
	private double m_EyeThrust;
	private double m_EyeRotate;
	private Relay LED;

	/**
	 * This function implements a simple method of generating driving and steering commands
	 * based on the tracking data from a Rasperry Pi writing to a network table under the name Eye.
	 */
	public static Eye getInstance() {
		if(instance == null){
			instance = new Eye();
		}
		return instance;
	}

	private Eye(){
		m_EyeHasValidTarget = false;
		m_EyeThrust = 0.0;
		m_EyeRotate = 0.0;

		yawPID = new PID(Constants.yawKP, Constants.yawKI, Constants.yawKD, Constants.yawTolerance);
		yawPID.setTarget(Constants.desiredTargetYaw);

		sizePID = new PID(Constants.sizeKP, Constants.sizeKI, Constants.sizeKD, Constants.sizeTolerance);
		sizePID.setTarget(Constants.desiredTargetSizeDiscLow);

		LED = Robot.m_cfg.getLED();
	}

	public void updateTracking(){
		m_EyeHasValidTarget = NetworkTableInstance.getDefault().getTable("ChickenVision").getEntry("tapeDetected").getBoolean(false);
		targetSizeObserved = NetworkTableInstance.getDefault().getTable("ChickenVision").getEntry("tapeSize").getDouble(Constants.desiredTargetSizeDiscLow);
		if(targetYawError != NetworkTableInstance.getDefault().getTable("ChickenVision").getEntry("tapeYaw").getDouble(Constants.desiredTargetYaw)){
			captured = false;
		}
		if(!captured){
			targetYawError = NetworkTableInstance.getDefault().getTable("ChickenVision").getEntry("tapeYaw").getDouble(Constants.desiredTargetYaw);
			captured = true;
			if (!m_EyeHasValidTarget){
				m_EyeThrust = 0.0;
				m_EyeRotate = 0.0;
				captured = false;
				return;
			}
			Robot.m_driveTrain.setTargetAngle(Robot.m_driveTrain.getHeading() + getYawError());
			// setDesiredTargetSize(Robot.m_arm.getCurrentState());
			// m_EyeRotate = yawPID.calculate(targetYawError);
			// m_EyeThrust = sizePID.calculate(targetSizeObserved);

			// don't let the robot drive too fast into the goal
			if (m_EyeThrust > Constants.throttle)
			{
				m_EyeThrust = Constants.throttle;
			}
		}
	}

	public double getDistanceCorrection(){
		return m_EyeThrust;
	}

	public double getHeadingCorrection(){
		return m_EyeRotate;
	}

	public double  getYawError(){
		return targetYawError;
	}

	public double  getSizeError(){
		return targetSizeObserved;
	}

	public boolean validTarget(){
		return m_EyeHasValidTarget;
	}

	public boolean targetAligned(){
		return Math.abs(targetYawError) <= Constants.yawTolerance;
	}

	public boolean targetCloseEnough(){
		return Math.abs(targetSizeObserved - sizePID.getTarget()) <= Constants.yawTolerance;
	}

	public void setDesiredTargetSize(Arm.State position){
		switch (position) {
			case DISC_LOW:
			case DISC_HP:
				sizePID.setTarget(Constants.desiredTargetSizeDiscLow);
				break;
			case DISC_MID:
				sizePID.setTarget(Constants.desiredTargetSizeDiscMid);
			case DISC_HIGH:
				sizePID.setTarget(Constants.desiredTargetSizeDiscHigh);
				break;
			case BALL_LOW:
				sizePID.setTarget(Constants.desiredTargetSizeBallLow);
				break;
			case BALL_MID:
				sizePID.setTarget(Constants.desiredTargetSizeBallMid);
			case BALL_HIGH:
				sizePID.setTarget(Constants.desiredTargetSizeBallHigh);
				break;
			case BALL_CARGO:
				sizePID.setTarget(Constants.desiredTargetSizeBallCargo);
				break;
			default:
				sizePID.setTarget(Constants.desiredTargetSizeDiscLow);
				break;
		}
	}

	public void look(){
		LED.set(Value.kOn);
	}

	public void close(){
		captured = false;
		yawPID.reset();
		sizePID.reset();
		LED.set(Value.kOff);
	}
}

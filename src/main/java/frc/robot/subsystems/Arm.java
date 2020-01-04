/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.commands.ArmOpenLoop;
import frc.robot.commands.ArmWristTeleopControl;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.DaveDigitalInput;
import frc.robot.utils.MathUtils;
import frc.robot.utils.PID;


	public class Arm extends Subsystem {
	private static Arm instance;

	private VictorSPX mLeft;
	private VictorSPX mRight;
	private Encoder mEnc;
	private DaveDigitalInput mHome;

	private State currentState;
	private State targetState;
	private State lastState;

	private PID mPID;
	private double output;

	private ShuffleboardTab tab;
	private NetworkTableEntry armAngle;
	private NetworkTableEntry armOutputEntry;
	private NetworkTableEntry armAtHome;
	private NetworkTableEntry currentStateData;
	private NetworkTableEntry targetStateData;
	private NetworkTableEntry targetPosition;
	public enum State{
		OPEN_LOOP("OPEN_LOOP", 0),
		DISC_HP("DISC_HP", Constants.armIntakeDiscPos),
		BALL_HP("BALL_HP", Constants.armIntakeBallHP), 
		BALL_FLOOR("BALL_FLOOR", Constants.armIntakeBallFloor),
		LOADED_STOWED("LOADED_STOWED", Constants.armHoldingGamePiece),
		STOWED("STOWED", Constants.armStowedPos),
		DISC_LOW("DISC_LOW", Constants.armScoreDisc1),
		DISC_MID("DISC_MID", Constants.armScoreDisc2),
		DISC_HIGH("DISC_HIGH", Constants.armScoreDisc3),
		BALL_LOW("BALL_LOW", Constants.armScoreBall1),
		BALL_MID("BALL_MID", Constants.armScoreBall2),
		BALL_HIGH("BALL_HIGH", Constants.armScoreBall3),
		BALL_CARGO("BALL_CARGO", Constants.armScoreBallCargo),
		CLIMB_SETUP("CLIMB_SETUP", Constants.armClimbSetup),
		CLIMB("CLIMB", Constants.armClimb),
		DISABLED("DISABLED", 0);

		private String name;
		private double setpoint;
		private State(String nString, double num){
			name = nString;
			setpoint = num;
		}
		public String getName(){
			return name;
		}
		public double getSetpoint() {
			return setpoint;
		}
	}

	public static Arm getInstance() {
		if (instance == null) {
			instance = new Arm(Robot.m_cfg);
		}
		
		return instance;
	}
	
	private Arm(Config cfg){
		mLeft = cfg.getArmMotorLeft();
		mRight = cfg.getArmMotorRight();
		mEnc = cfg.getArmEncoder();
		mHome = cfg.getArmHomeSensor();
		currentState = State.STOWED;
		targetState = State.STOWED;
		lastState = State.STOWED;

		mPID = new PID(Constants.armKP,Constants.armKI,Constants.armKD, Constants.armTol);
		output = 0.0;

		mEnc.reset();

		tab = Shuffleboard.getTab(Constants.SuperStructureTabName);
		tab.add("Encoder", mEnc);

		armAngle = tab.add("Arm Angle", 0).getEntry();
		armAtHome = tab.add("Arm at Home", false).getEntry();
		currentStateData = tab.add("Arm Current State", currentState.getName()).getEntry();
		targetStateData = tab.add("Arm Target State", targetState.getName()).getEntry();
		targetPosition = tab.add("Target Arm Position",  targetState.getSetpoint()).getEntry();
		armOutputEntry = tab.add("Arm output",  output).getEntry();
	}  
 
  @Override
	public void initDefaultCommand() {
		setDefaultCommand(new ArmWristTeleopControl());
	}
	
  	public void handleState(){
		switch (targetState){
			case DISC_HP:
			case BALL_HP:
			case BALL_FLOOR:
			case LOADED_STOWED:
			case STOWED:
			case DISC_LOW:
			case DISC_MID:
			case DISC_HIGH:
			case BALL_LOW:
			case BALL_MID:
			case BALL_HIGH:
			case BALL_CARGO:
			case CLIMB_SETUP:
			case CLIMB:
				handleCLOSED_LOOP();
				break;
			case DISABLED:
				handleDISABLED();
				break;
			case OPEN_LOOP:
				handleOPEN_LOOP();
				break;
		}
    }
	private void handleOPEN_LOOP(){
		currentState = State.OPEN_LOOP;
	}
	private void handleCLOSED_LOOP(){
		mPID.setTarget(targetState.getSetpoint());
		setOutput(mPID.calculate(getAngle()));
		if(Math.abs(getAngle() - targetState.getSetpoint()) <= 5){
			currentState = targetState;
		}
	}
	private void handleDISABLED(){
		if(currentState != State.DISABLED){
			lastState = currentState;
		}
		currentState = State.DISABLED;
		setOutput(0);
	}
	public State getCurrentState(){
		return currentState;
	}
	public State getTargetState(){
		return targetState;
	}
	public State getLastState(){
		return lastState;
	}
	public void setTargetState(State state){
		targetState = state;
	}
	public void runMotors (double power){
		mLeft.set(ControlMode.PercentOutput, -power);
		mRight.set(ControlMode.PercentOutput, power);
	}
	public double getAngle(){
		return mEnc.getDistance() + Constants.armHomeOffset;
	}
	public double getAngularVelocity(){
		return mEnc.getRate();
	}
	public boolean isAtHome(){
		return !mHome.get();
	}
	public boolean EnteredHome(){
		return mHome.getTriggered();
	}
	public boolean staggerReady(){
		if(targetState == State.DISC_MID)
			return Math.abs(getAngle() - targetState.getSetpoint()) <= Constants.armStaggerThresholdMid;
		else
			return Math.abs(getAngle() - targetState.getSetpoint()) <= Constants.armStaggerThreshold;
	}
	public void zeroEncoder() {
		mEnc.reset();
	}
	public void setOutput(double power){
		output = power;

	}
	private void sendToDashboard(){
		armAngle.setDouble(getAngle());
		armAtHome.setBoolean(isAtHome());
		currentStateData.setString(currentState.getName());
		targetStateData.setString(targetState.getName());
		targetPosition.setDouble(targetState.getSetpoint());
		armOutputEntry.setDouble(output);
	}
	
	@Override
	public void periodic() {
		handleState();
		mHome.update();
		if(targetState == State.CLIMB){
			if(output <= -0.65) {
				output = -0.65;
			}
		}
		if(targetState == State.BALL_LOW) {
			if (Math.abs(output) > 0.4) {
				output = Math.signum(output)*0.4;
			}
		}		
		if (getAngle() <= Constants.armThrottleThreshold && output < 0 && targetState==State.STOWED){
			double alpha = MathUtils.unlerp(Constants.armThrottleThreshold, Constants.armHomeOffset, getAngle());
			if(output <= -0.5) {
				output = -0.5;
			}//MathUtils.lerp(0.7, 0.4, alpha);
		}
		if(EnteredHome() && (currentState == State.OPEN_LOOP || currentState == State.DISABLED)){
			zeroEncoder();
		}
		
		if(isAtHome()){
			if(output <= 0) {
				output = 0;
			}
		}
		if(getAngle() <= Constants.armHomeOffset){
			if(currentState == State.OPEN_LOOP){
				if(output <= -0.3) {
					output = -0.3;
				}
			}
			else{
				if(output <= 0) {
					output = 0;
				}
			}
		}
		if (getAngle() >= Constants.armUpperLimit){
			if(output > 0) {
				output = 0;
			}
		}
		runMotors(output);
		sendToDashboard();
	}
}

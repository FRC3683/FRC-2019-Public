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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.PID;;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
	public static Wrist instance; 

	private VictorSPX mLeft;
	private VictorSPX mRight;
	private PID wristPID;
	private AnalogPotentiometer mPot;
	private double output;

	private double potOffset;
  
	public enum State{
		OPEN_LOOP("OPEN_LOOP", 0),
		DISC_HP("DISC_HP", Constants.wristIntakeDiscPos),
		BALL_HP("BALL_HP", Constants.wristIntakeBallHP), 
		BALL_FLOOR("BALL_FLOOR", Constants.wristIntakeBallFloor),
		LOADED_STOWED("LOADED_STOWED", Constants.wristHoldingGamePiece),
		STOWED("STOWED", Constants.wristStowedPos),
		DISC_LOW("DISC_LOW", Constants.wristScoreDisc1),
		DISC_MID("DISC_MID", Constants.wristScoreDisc2),
		DISC_HIGH("DISC_HIGH", Constants.wristScoreDisc3),
		BALL_LOW("BALL_LOW", Constants.wristScoreBall1),
		BALL_MID("BALL_MID", Constants.wristScoreBall2),
		BALL_HIGH("BALL_HIGH", Constants.wristScoreBall3),
		BALL_CARGO("BALL_CARGO", Constants.wristScoreBallCargo),
		CLIMB_SETUP("CLIMB_SETUP", Constants.wristClimbSetup),
		CLIMB("CLIMB", Constants.wristClimb),
		DISABLED("DISABLED", 0);
		
		private String name;
		private double setpoint;
		private State(String nString, double num){
			name = nString;
			setpoint = num;
		}
		public String getName() {
			return name;
		}
		public double getSetpoint() {
			return setpoint;
		}
	}
	private State currentState;
	private State targetState;
	private State lastState;
	
	ShuffleboardTab tab;
	NetworkTableEntry wristout;
	NetworkTableEntry wristangle;
	NetworkTableEntry currentWristStateEntry;
	NetworkTableEntry targetStateEntry;
	NetworkTableEntry targetPosition;

	public static Wrist getInstance(){
		if (instance == null) {
			instance = new Wrist(Robot.m_cfg);
		}
		return instance;
	}
	public Wrist(Config cfg){
		mLeft = cfg.getWristMotorLeft();
		mRight = cfg.getWristMotorRight();
		mPot = cfg.getHandPot();
		currentState = State.STOWED;
		targetState = State.STOWED;
		lastState = State.STOWED;

		wristPID = new PID(Constants.wristkP, Constants.wristkI, Constants.wristkD, Constants.wristkTol);
		output = 0.0;

		tab = Shuffleboard.getTab(Constants.SuperStructureTabName);
		tab.add("Wrist Pot", mPot);

		potOffset = 0.0;

		wristout = tab.add("Wrist Output", output).getEntry();
		wristangle = tab.add("Wrist Angle", getAngle()).getEntry();
		currentWristStateEntry = tab.add("Current Wrist State", currentState.getName()).getEntry();
		targetStateEntry = tab.add("Target Wrist State",  targetState.getName()).getEntry();
		targetPosition = tab.add("Target Wrist Position",  targetState.getSetpoint()).getEntry();
	}
  
	@Override
  	public void initDefaultCommand() {
			//setDefaultCommand(new WristOpenLoop());
		//setDefaultCommand(new WristTeleOpControl());
	}

	private void handleState() {
		switch(targetState) {
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
	private void handleDISABLED() {
		if(currentState != State.DISABLED){
			lastState = currentState;
		}
		currentState = targetState;
		setOutput(0);
	}
	private void handleOPEN_LOOP() {
		currentState = State.OPEN_LOOP;
	}
	private void handleCLOSED_LOOP(){
		double desiredAngle = targetState.getSetpoint();		
		//double desiredAngle = targetState.getSetpoint() + potOfxfset;
		wristPID.setTarget(desiredAngle);
		setOutput(wristPID.calculate(getAngle()));
		if(Math.abs(getAngle() - desiredAngle) <= 5){
			currentState = targetState;
		}
	}
	public State getCurrentState() {
		return currentState; 
	}
	public State getTargetState() {
		return targetState; 
	}
	public State getLastState(){
		return lastState;
	}
	public void setTargetState(State state) {
		targetState = state;
	}
	public void setOutput(double output){
		this.output = output;
	}
	private void runWristMotor(double power){
		mLeft.set(ControlMode.PercentOutput, -power);
		mRight.set(ControlMode.PercentOutput, power);
	}
	
	public double getAngle() {
		return -((mPot.get() - Constants.wristPotRawOffset)* Constants.wristPotValueToAngle + Constants.wristPotAngleOffset);
	}

	public boolean staggerReady(){
		if(Robot.exit_climb){
			return Math.abs(getAngle() - targetState.getSetpoint()) <= Constants.wristStaggerThresholdExitClimb;
		}
		else{
			return Math.abs(getAngle() - targetState.getSetpoint()) <= Constants.wristStaggerThreshold;
		}
	}

	public void incrementPotOffset(){
		this.potOffset++;
	}

	public void decrementPotOffset(){
		this.potOffset--;
	}

	public void zeroPotOffset(){
		this.potOffset = 0.0;
	}

	private void calculatefV(){
		double lengthToCenterOfMass =0;
		double weight =0;
		double angle=0;

		double internalResistance =0;
		double torque =0;
		double motorStallCurrent =0; 
		double gearReduction =0;
		
		double Kt = (torque/motorStallCurrent);
		double Kf= ((lengthToCenterOfMass*weight*internalResistance)/Kt)* Math.cos(Math.toRadians(angle));

	}

	@Override
	public void periodic() {
		//calculate FeedForward coefficient
		calculatefV();

		//Update states, set output values
		handleState();
		if(currentState == State.STOWED){
			Robot.exit_climb = false;
		}

		//Run motors
		runWristMotor(0.7*output);
		
		//Send system and sensor data to dashboard
		sendToDashboard();
	}

	private void sendToDashboard(){
		wristout.setDouble(output);
		wristangle.setDouble(getAngle());
		targetPosition.setDouble(targetState.getSetpoint());
		currentWristStateEntry.setString(currentState.getName());
		targetStateEntry.setString(targetState.getName());
	}
}

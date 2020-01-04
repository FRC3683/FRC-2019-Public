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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.commands.HandOpenLoop;
import frc.robot.commands.HandTeleOpControl;
import frc.robot.config.Config;
import frc.robot.config.Constants;

/**
 * Add your docs here.
 */
public class Hand extends Subsystem {
	private static Hand instance;

	private VictorSPX mRoller;
	private Solenoid pushPiston; //punch
	private Solenoid discPiston; //disc
	private Solenoid jawPiston;

	private double shootSpeed = 1;
	private double intakeSpeedFloor = -0.7;
	private double intakeHoldSpeed = -0.15;


	private double intakeOutput;

	ShuffleboardTab tab;

	public enum IntakeState { //for controlling roller
		DISABLED("DISABLED"),
		STOPPED("STOPPED"),
		INTAKE_FLOOR("INTAKE_FLOOR"),
		SHOOT("SHOOT"),
		HOLDING_BALL("HOLDING BALL"),
		OPEN_LOOP("OPEN_LOOP");
		private String name;
		private IntakeState(String nString){
			name = nString;
		}
		public String getName() {
			return name;
		}
	}
	public enum HandState{
		OPEN_LOOP("OPEN_LOOP"),
		PICKUP_DISC("PICKUP_DISC"),
		JUST_SHOT("JUST_SHOT"),
		BALL_MODE("BALL_MODE"),
		HOLDING_DISC("HOLDING_DISC"),
		DISC_DETECTED("DISC_DETECTED"),
		SMOKING_DISC("SMOKING_DISC"), 
		EMPTY_DISC("EMPTY_DISC");
		private String name;
		private HandState(String nString){
			name = nString;
		}
		public String getName() {
			return name;
		}
	}
	private IntakeState intakeState;

	private HandState handState;

	NetworkTableEntry handStateEntry;
	NetworkTableEntry intakeStateEntry;
	NetworkTableEntry intakeOutputEntry;

	private double shotTime;
	private boolean shotTimeCaptured;

	public static Hand getInstance(){
		if (instance == null) {
			instance = new Hand(Robot.m_cfg);
		}
		return instance;
	}
	private Hand(Config cfg){
		mRoller = cfg.getHandRoller();
		pushPiston = cfg.getHandPushPiston();
		discPiston = cfg.getHandDiscPiston();
		jawPiston = cfg.getHandJawPiston();
		
		intakeOutput = 0.0;

		handState = HandState.EMPTY_DISC;
		intakeState = IntakeState.STOPPED;

		shotTime = Timer.getFPGATimestamp();
		shotTimeCaptured = false;

		tab = Shuffleboard.getTab(Constants.SuperStructureTabName);
		tab.add("Punch", pushPiston);
		tab.add("Fingers", discPiston);
		tab.add("Jaw", jawPiston);

		handStateEntry = tab.add("Hand State",  handState.getName()).getEntry();
		intakeStateEntry = tab.add("Intake State", intakeState.getName()).getEntry();
		intakeOutputEntry = tab.add("Intake Output", 0.0).getEntry();

	}
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new HandTeleOpControl());
	}
	private void handleIntakeState() {
			switch(intakeState) {
				case STOPPED:
					handleSTOPPED();
					break;
				case INTAKE_FLOOR:
					handleINTAKE_FLOOR();
					break;
				case SHOOT:
					handleSHOOT();
					break;
				case DISABLED:
					handleDISABLED();
					break;
				case HOLDING_BALL:
					handleHOLDING_BALL();
					break;
				case OPEN_LOOP:
					handleOPEN_LOOP();
					break;
			}
	}
	private void handleHandState() {
		switch(handState) {
			case EMPTY_DISC:
				handleEMPTY_DISC();
				break;
			case PICKUP_DISC:
				handlePICKUP_DISC();
				break;
			case JUST_SHOT:
				handleJUST_SHOT();
				break;
			case BALL_MODE:
				handleBALL_MODE();
				break;
			case HOLDING_DISC:
				handleHOLDING_DISC();
				break;
			case DISC_DETECTED:
				handleDISC_DETECTED();
				break;
			case SMOKING_DISC:
				handleSMOKING_DISC();
				break;
			case OPEN_LOOP:
				handleOPEN_LOOP();
				break;
		}
	}
	
	private void handleStates() {
		handleIntakeState();
		handleHandState();
	}
	
	private void handleOPEN_LOOP() {
	}
	private void handleSTOPPED() {
		intakeOutput = 0.0;
	}
	private void handleHOLDING_BALL() {
		intakeOutput = intakeHoldSpeed;
	}
	private void handleINTAKE_FLOOR() {
		intakeOutput = intakeSpeedFloor;
	}
	private void handleSHOOT() {
		intakeOutput = shootSpeed;
	}
	private void handleDISABLED() {
		intakeOutput = 0.0;
	}
	
	private void handleEMPTY_DISC() {
		closeJaw();
		retractPunch();
		closeDisc();
		shotTimeCaptured = true;
	}
	private void handlePICKUP_DISC() {
		closeJaw();
		retractPunch();
		openDisc();
	}
	private void handleJUST_SHOT() {
		if(!shotTimeCaptured){
			shotTime = Timer.getFPGATimestamp();
			shotTimeCaptured = true;
		}
		closeJaw();
		openDisc();
		if(shotTimeCaptured && Timer.getFPGATimestamp() - shotTime >= Constants.discShotDelay){
			punchDisc();
		}
	}
	private void handleBALL_MODE() {
		openJaw();
		retractPunch();
		closeDisc();
	}
	private void handleHOLDING_DISC() {
		closeJaw();
		retractPunch();
		closeDisc();
	}
	private void handleDISC_DETECTED() {
		closeJaw();
		retractPunch();
		openDisc();
	}
	private void handleSMOKING_DISC() {
		closeJaw();
		punchDisc();
		openDisc();

	}
	private void runRoller(double speed) {
		mRoller.set(ControlMode.PercentOutput, speed);
	}
	public void openJaw() {
		jawPiston.set(true);
	}
	public void closeJaw() {
		jawPiston.set(false);
	}
	public void closeDisc() {
		discPiston.set(false);
	}
	public void openDisc() {
		discPiston.set(true);
	}
	public void punchDisc() {
		pushPiston.set(true);
	}
	public void retractPunch() {
		pushPiston.set(false);
	}
	public void setIntakeOutput(double output){
		intakeOutput = output;
	}
	//Getters and Setters
	public HandState getHandState() {
		return handState; 
	}
	public void setHandState(HandState state) {
		handState = state;
	}
	public IntakeState getIntakeState() {
		return intakeState;
	}
	public void setIntakeState(IntakeState state) {
		intakeState = state;
	}

	@Override
	public void periodic() {
		//Update states, set output values
		handleStates();

		//Run motors
		runRoller(intakeOutput);
		
		//Send system and sensor data to dashboard
		sendToDashboard();
	}

	private void sendToDashboard(){
		handStateEntry.setString(handState.getName());
		intakeStateEntry.setString(intakeState.getName());
		intakeOutputEntry.setDouble(intakeOutput);
	}
}
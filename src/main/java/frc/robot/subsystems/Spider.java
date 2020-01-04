/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.commands.SpiderTeleOpControl;
import frc.robot.config.Config;

/**
 * Add your docs here.
 */
public class Spider extends Subsystem {
	public static Spider instance;
	private Config cfg;

	private Solenoid mPiston1;
	private Solenoid mPiston2;

	private ShuffleboardTab tab;
	private NetworkTableEntry stateEntry;

	public enum State{
		DISENGAGED("DISENGAGED"),
		ENGAGED("ENGAGED");

		private String name;
		private State(String nString){
			name = nString;
		}

		public String getName(){
			return name;
		}
	}

	private State state;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new SpiderTeleOpControl());
	}

	public State getState() {
		return state;
	}

	public void setState(State newState) {
		state = newState;
	}

	public static Spider getInstance() {
		if (instance == null) {
			instance = new Spider(Robot.m_cfg);
		}
		
		return instance;
	}
	private Spider(Config cfg){
		mPiston1 = cfg.getClimbPiston1();
		mPiston2 = cfg.getClimbPiston2();

		state = State.DISENGAGED;

		
		tab = Shuffleboard.getTab("Drive");
		stateEntry = tab.add("Spider State", state.getName()).getEntry();
	}

	public void handleStates(){
		switch (state) {
			case DISENGAGED:
				handleDISENGAGED();
				break;
			case ENGAGED:
				handleENGAGED();
				break;
		}
	}

	private void handleDISENGAGED(){
		mPiston1.set(false);
		mPiston2.set(false);
	}

	private void handleENGAGED(){
		mPiston1.set(true);
		mPiston2.set(true);

	}

	@Override
	public void periodic() {
		handleStates();
	}

	public void sendToDashboard(){
		stateEntry.setString(state.getName());
	}
}

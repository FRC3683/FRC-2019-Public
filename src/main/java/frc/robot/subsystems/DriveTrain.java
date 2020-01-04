/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.commands.Drive;
import frc.robot.commands.Drive.DriveMode;
import frc.robot.config.Constants;
import frc.robot.utils.BNO055;
import frc.robot.utils.DaveAccelerometer;
import frc.robot.utils.MathUtils;
import frc.robot.utils.PID;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
	private static DriveTrain instance;

	private VictorSPX mLeft1;
	private VictorSPX mLeft2;
	private VictorSPX mRight1;
	private VictorSPX mRight2;
	private DoubleSolenoid mShifter;

	private Encoder leftEnc;
	private Encoder rightEnc;

	private BNO055 mGyro;
	private DaveAccelerometer mAccelerometer;

	private double leftOutput;
	private double rightOutput;
	private boolean headingLocked;

	private PID straightPID;
	private PID turnPID;
	private double throttle;

	private ShuffleboardTab tab;
	private NetworkTableEntry headingEntry;
	private NetworkTableEntry rollEntry;
	private NetworkTableEntry pitchEntry;
	private NetworkTableEntry encLeftEntry;
	private NetworkTableEntry encRightEntry;
	private NetworkTableEntry leftOutEntry;
	private NetworkTableEntry rightOutEntry;
	private NetworkTableEntry stateEntry;
	private NetworkTableEntry targetEntry;
	private NetworkTableEntry offsetEntry;
	private NetworkTableEntry onTargetEntry;

	private double offsetHeadingAngle;
	private double offsetPitchAngle;
	private double offsetRollAngle;

	public enum State{
		HIGH_GEAR_CLOSED_LOOP("HIGH_GEAR_CLOSED_LOOP"),
		LOW_GEAR_CLOSED_LOOP("LOW_GEAR_CLOSED_LOOP"),
		HIGH_GEAR_OPEN_LOOP("HIGH_GEAR_OPEN_LOOP"),
		LOW_GEAR_OPEN_LOOP("LOW_GEAR_OPEN_LOOP"),
		DISABLED("DISABLED");

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
		setDefaultCommand(new Drive(DriveMode.ARCADE_TWO_STICK));
	}

	public static DriveTrain getInstance() {
		if(instance == null){
			instance = new DriveTrain();
		}
		return instance;
	}

	private DriveTrain(){
		mLeft1 = Robot.m_cfg.getDriveTrainLeft1();
		mLeft2 = Robot.m_cfg.getDriveTrainLeft2();
		mRight1 = Robot.m_cfg.getDriveTrainRight1();
		mRight2 = Robot.m_cfg.getDriveTrainRight2();
		mShifter = Robot.m_cfg.getDriveTrainShifter();

		leftEnc = Robot.m_cfg.getDriveTrainLeftEnc();
		rightEnc = Robot.m_cfg.getDriveTrainRightEnc();

		mGyro = Robot.m_cfg.getDriveTrainGyro();
		mAccelerometer = DaveAccelerometer.getInstance();

		offsetHeadingAngle = 0.0;
		offsetPitchAngle = 0.0;
		offsetRollAngle = 0.0;
		tab = Shuffleboard.getTab("Drive");
		
		straightPID = new PID(Constants.driveStraightKP, Constants.driveStraightKI, Constants.driveStraightKD, Constants.driveStraightTolerance);
		turnPID = new PID(Constants.driveTurnKP, Constants.driveTurnKI, Constants.driveTurnKD, Constants.driveTurnTolerance);
   
		leftOutput = 0;
		rightOutput = 0;

		tab.add("Left Encoder", leftEnc);
		tab.add("Right Encoder", rightEnc);
		
		headingEntry = tab.add("Heading", 0).getEntry();
		rollEntry = tab.add("roll", 0).getEntry();
		pitchEntry = tab.add("Pitch", 0).getEntry();
		encLeftEntry = tab.add("LeftEnc", 0).getEntry();
		encRightEntry = tab.add("RightEnc", 0).getEntry();
		stateEntry = tab.add("State", "disabled").getEntry();
		targetEntry = tab.add("target Angle", turnPID.getTarget()).getEntry();
		leftOutEntry = tab.add("Thrust", leftOutput).getEntry();
		rightOutEntry = tab.add("Rotate", rightOutput).getEntry();
		offsetEntry = tab.add("Offset", offsetHeadingAngle).getEntry();
		onTargetEntry = tab.add("onTargetAngle", false).getEntry();
	}

	public void setState(State s){
		state = s;
	}
	
	public State getState(){
		return state;
	}

	private void handleStates(){
		switch(state){
			case LOW_GEAR_CLOSED_LOOP:
				handleLOW_GEAR_CLOSED_LOOP();
				break;
			case LOW_GEAR_OPEN_LOOP:
				handleLOW_GEAR_OPEN_LOOP();	
				break;
			case HIGH_GEAR_CLOSED_LOOP:
				handleHIGH_GEAR_CLOSED_LOOP();
				break;
			case HIGH_GEAR_OPEN_LOOP:
				handleHIGH_GEAR_OPEN_LOOP();
				break;
			case DISABLED:
				handleDISABLED();
				break;
		}
	}

	private void handleLOW_GEAR_CLOSED_LOOP() {
		mShifter.set(Value.kForward);
		brakeMode();

		double thrust = straightPID.calculate(getDistance());
		double rotate = turnPID.calculate(getHeading());
		//if(!onTargetAngle()){
		//	rotate = -turnPID.calculate(getHeading());
		//}
		setLeft(MathUtils.calcLeftDrive(thrust, rotate));
		setRight(MathUtils.calcRightDrive(thrust, rotate));
		targetEntry.setDouble(turnPID.getTarget());
		
		leftOutEntry.setDouble(thrust);
		rightOutEntry.setDouble(rotate);
	}

	private void handleLOW_GEAR_OPEN_LOOP() {
		mShifter.set(Value.kForward);
		coastMode();
		
		double rotate = 0;
		if(headingLocked){
			rotate = turnPID.calculate(getHeading());
		}
		else{
			//setTargetAngle(getHeading());
		}

		setLeft(leftOutput+rotate);
		setRight(rightOutput+rotate);
	}

	private void handleHIGH_GEAR_CLOSED_LOOP() {
		mShifter.set(Value.kReverse);
		brakeMode();

		double thrust = 0.0; //-0.2 * straightPID.calculate(getDistance());
		double rotate = 0.0;
		if(!onTargetAngle()){
			rotate = turnPID.calculate(getHeading());
		}
		setLeft(MathUtils.calcLeftDrive(thrust, rotate));
		setRight(MathUtils.calcRightDrive(thrust, rotate));
		
		targetEntry.setDouble(turnPID.getTarget());
		
		leftOutEntry.setDouble(thrust);
		rightOutEntry.setDouble(rotate);
	}

	private void handleHIGH_GEAR_OPEN_LOOP() {
		mShifter.set(Value.kReverse);
		coastMode();
	}
	
	private void handleDISABLED() {
		mShifter.set(Value.kReverse);
		coastMode();
		setLeft(0);
		setRight(0);
	}

	public void setLeft(double power){
		leftOutput = power;
	}

	public void setRight(double power){
		rightOutput = power;
	}

	private void coastMode(){
		mLeft1.setNeutralMode(NeutralMode.Coast);
		mLeft2.setNeutralMode(NeutralMode.Coast);
		mRight1.setNeutralMode(NeutralMode.Coast);
		mRight2.setNeutralMode(NeutralMode.Coast);
	}

	private void brakeMode(){
		mLeft1.setNeutralMode(NeutralMode.Brake);
		mLeft2.setNeutralMode(NeutralMode.Brake);
		mRight1.setNeutralMode(NeutralMode.Brake);
		mRight2.setNeutralMode(NeutralMode.Brake);
	}

	private void runLeft(double power){
		mLeft1.set(ControlMode.PercentOutput, power);
	}

	private void runRight(double power){
		mRight1.set(ControlMode.PercentOutput, power);
	}

	private int getLeftRaw(){
		return leftEnc.getRaw();
	}

	private int getRightRaw(){
		return rightEnc.getRaw();
	}

	public double getHeading(){
		return mGyro.getHeading() - offsetHeadingAngle;
	}

	public double getRoll(){
		return mGyro.getVector()[1] - offsetRollAngle;
	}

	public double getPitch(){
		return mGyro.getVector()[2] - offsetPitchAngle;
	}

	public double getDistance(){
		return(MathUtils.average(leftEnc.getDistance(), rightEnc.getDistance()));
	}

	public void setTargetDistance(double dist){
		straightPID.setTarget(dist);
	}

	public void setTargetAngle(double deg){
		turnPID.setTarget(deg);
		targetEntry.setDouble(deg);
	}

	public void zeroGyro(){
		System.out.println("Gyro Zeroed");
		offsetHeadingAngle = getHeading();
		offsetPitchAngle = getPitch();
		offsetRollAngle = getRoll();
	}

	public void zeroEncoders(){
		leftEnc.reset();
		rightEnc.reset();
	}

	public void zeroAccelerometer(){
		mAccelerometer.resetVelocities();
	}

	public boolean onTargetPosition(){
		return straightPID.isDone();
	}

	public boolean onTargetAngle(){
		return turnPID.isDone() || Math.abs(getHeading()-turnPID.getTarget()) <= Constants.driveTurnTolerance;
	}

	public double getThrottle(){
		return throttle;
	}

	public void setThrottle(double throttle){
		this.throttle = throttle;
	}

	public void setHeadingLock(boolean lock){
		headingLocked = lock;
	}

	public double visionRotateAssist(){
		double value = turnPID.calculate(getHeading());
		if(Robot.m_driveTrain.onTargetAngle()){
			return 0;
		}		
		return Math.copySign(Math.abs(value) > 0.5 ? 0.5 : Math.abs(value), value);
	}

	@Override
	public void periodic() {
		handleStates();
		sendToDashboard();
		mAccelerometer.updateVelocities();
		
		runLeft(leftOutput);
		runRight(rightOutput);
	}

	private void sendToDashboard() {
		headingEntry.setDouble(getHeading());
		encLeftEntry.setDouble(getLeftRaw());
		encRightEntry.setDouble(getRightRaw());
		offsetEntry.setDouble(offsetHeadingAngle);
		stateEntry.setString(state.getName());
		onTargetEntry.setBoolean(onTargetAngle());
	}
}
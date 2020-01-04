/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.config;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.utils.BNO055;
import frc.robot.utils.DaveDigitalInput;

/**
 * Add your docs here.
 */
public class Config {
    private static Config instance;

    // declare all your actuators
    private VictorSPX driveMotorLeft1;
    private VictorSPX driveMotorLeft2;
    private VictorSPX driveMotorRight1;
    private VictorSPX driveMotorRight2;
    private DoubleSolenoid driveShifter;

    //declare all your sensors
    private Encoder driveLeftEnc;
    private Encoder driveRightEnc;
    private BNO055 driveGyro;
    private VictorSPX handRoller;
    private VictorSPX wristMotorLeft;
    private VictorSPX wristMotorRight;
    private Solenoid handPushPiston;
    private Solenoid handDiscPiston;
    private Solenoid handJawPiston;

    private AnalogPotentiometer handPot;
    private VictorSPX armMotorLeft;
    private VictorSPX armMotorRight;

    //declare all your sensors
    private Encoder armEncoder;
    private DaveDigitalInput armHomeSensor;

    private Solenoid climbPiston1;
    private Solenoid climbPiston2;

    private Relay led;
    private boolean compBot;

    public static Config getInstance() {
        if (instance == null) {
            instance = new Config();
        }
        return instance;
    }

    private Config (){
        //initialize all actuators
        driveMotorLeft1 = new VictorSPX(5);
        driveMotorLeft2 = new VictorSPX(6);
        driveMotorRight1 = new VictorSPX(0);
        driveMotorRight2 = new VictorSPX(1);
        driveShifter = new DoubleSolenoid(5,6);

        //initialize all sensors
        driveLeftEnc = new Encoder(2, 3);
        driveLeftEnc.setDistancePerPulse(1.0 / Constants.driveTicksPerInchLeft);
        driveLeftEnc.setReverseDirection(false);
        driveRightEnc = new Encoder(0, 1);
        driveRightEnc.setDistancePerPulse(1.0 / Constants.driveTicksPerInchRight);
        driveRightEnc.setReverseDirection(true);

        led = new Relay(1,Direction.kForward);
        armMotorLeft = new VictorSPX(7);
        armMotorRight = new VictorSPX(8);
        wristMotorLeft = new VictorSPX(4);
        wristMotorRight = new VictorSPX(2);
        handRoller = new VictorSPX(3);

        handPushPiston = new Solenoid(1);
        handDiscPiston = new Solenoid(2);
        handJawPiston = new Solenoid(3);
            
        armMotorLeft.setNeutralMode(NeutralMode.Brake);
        armMotorRight.setNeutralMode(NeutralMode.Brake);
        wristMotorLeft.setNeutralMode(NeutralMode.Brake);
        wristMotorRight.setNeutralMode(NeutralMode.Brake);
            // shifter 5,6
            //lock 0
            //pto 4
            //initialize all sensors
        handPot = new AnalogPotentiometer(3);
        armEncoder = new Encoder(4,5);
        armHomeSensor = new DaveDigitalInput(6);
        
        armEncoder.setDistancePerPulse(1/Constants.armEncoderTicksPerDegree);
      
        driveGyro = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
        BNO055.vector_type_t.VECTOR_EULER,
        I2C.Port.kOnboard,
        BNO055.BNO055_ADDRESS_A);

        
		FileInputStream fileIn = null;

		try {
			fileIn = new FileInputStream("/robot.properties");
			Properties prop = new Properties();
			prop.load(fileIn);
			String isCompString = prop.getProperty("isComp");
			if(isCompString == null){
				throw new IOException("isComp property is null");
			}
			compBot = isCompString.equals("true");
		} catch (Exception e) {
			System.out.println("Error finding property file. Assuming Comp bot!!!");
			compBot = true;
		} finally {
			if(fileIn != null){
				try {
					fileIn.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}

        driveMotorRight2.follow(driveMotorRight1);
        driveMotorLeft2.follow(driveMotorLeft1);

        driveMotorLeft1.configOpenloopRamp(0.25);
        driveMotorLeft2.configOpenloopRamp(0.25);
        driveMotorRight1.configOpenloopRamp(0.25);
        driveMotorRight2.configOpenloopRamp(0.25);

        climbPiston1 = new Solenoid(4);
        climbPiston2 = new Solenoid(0);
    }
    public VictorSPX getDriveTrainLeft1(){
        return driveMotorLeft1;
    }
    public VictorSPX getDriveTrainLeft2() {
        return driveMotorLeft2;
    }
    public VictorSPX getDriveTrainRight1(){
        return driveMotorRight1;
    }
    public VictorSPX getDriveTrainRight2(){
        return driveMotorRight2;
    }

    public DoubleSolenoid getDriveTrainShifter(){
        return driveShifter;
    }
    public VictorSPX getHandRoller() {
        return handRoller;
    }
    public VictorSPX getWristMotorLeft() {
        return wristMotorLeft;
    }
    public VictorSPX getWristMotorRight() {
        return wristMotorRight;
    }
    public Encoder getDriveTrainLeftEnc() {
        return driveLeftEnc;
    }
    public Encoder getDriveTrainRightEnc() {
        return driveRightEnc;
    }
    public BNO055 getDriveTrainGyro() {
        return driveGyro;
    }
    public Relay getLED() {
      return led;
    }
    public Solenoid getHandPushPiston() {
        return handPushPiston;
    }
    public Solenoid getHandDiscPiston() {
        return handDiscPiston;
    }
    public Solenoid getHandJawPiston() {
        return handJawPiston;
    }
    public AnalogPotentiometer getHandPot() {
        return handPot;
    }
    public VictorSPX getArmMotorLeft() {
        return armMotorLeft;
    }
    public VictorSPX getArmMotorRight() {
        return armMotorRight;
    }
    public Encoder getArmEncoder() {
        return armEncoder;
    }
    public DaveDigitalInput getArmHomeSensor(){
        return armHomeSensor;
    }
    public Solenoid getClimbPiston1() {
        return climbPiston1;
    }
    public Solenoid getClimbPiston2() {
        return climbPiston2;
    }
    public boolean isCompBot(){
        return compBot;
    }
}

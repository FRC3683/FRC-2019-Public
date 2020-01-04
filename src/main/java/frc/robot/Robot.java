/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.TestAuto;
import frc.robot.commands.autoCommands.autoCommandGroups.SandstormTeleop;
import frc.robot.config.Config;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.State;
import frc.robot.subsystems.Hand.HandState;
import frc.robot.vision.Eye;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Spider;
import frc.robot.subsystems.Wrist;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Config m_cfg;
  public static Eye m_eye;
  public static DriveTrain m_driveTrain;
  public static OI m_oi;
  public static Arm m_arm;
  public static Hand m_hand;
  public static Wrist m_wrist;
  public static Spider m_spider;
  public static boolean climbing;
  public static boolean exit_climb;
  boolean fromAuto;

  Command m_autonomousCommand;
  AutoSelector m_chooser;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_cfg = Config.getInstance();
    m_eye = Eye.getInstance();
    m_oi = OI.getInstance();
    m_chooser = new AutoSelector();
    fromAuto = false;
    //chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);
    m_driveTrain = DriveTrain.getInstance();
		m_arm = Arm.getInstance();
		m_wrist = Wrist.getInstance();
    m_hand = Hand.getInstance();
    m_spider = Spider.getInstance();

    CameraServer.getInstance().startAutomaticCapture();
    // chooser.addOption("My Auto", new MyAutoCommand());
    
    m_autonomousCommand = new SandstormTeleop();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_driveTrain.setState(State.DISABLED);
    m_wrist.setTargetState(Wrist.State.DISABLED);
    m_arm.setTargetState(Arm.State.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() { //m_chooser.getSelected();
    m_driveTrain.setState(State.LOW_GEAR_OPEN_LOOP);
    m_wrist.setTargetState(Wrist.State.STOWED);
    m_arm.setTargetState(Arm.State.STOWED);
    m_hand.setHandState(HandState.HOLDING_DISC);
    m_driveTrain.zeroEncoders();
    m_driveTrain.zeroGyro();
    fromAuto = true;
    
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
    //m_autonomousCommand = new TestAuto();
    // schedule the autonomous command (example)
    if(m_autonomousCommand != null){
      m_autonomousCommand.start();
    }
    Robot.climbing = false;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    m_driveTrain.zeroEncoders();
    climbing = false;
    exit_climb = false;

    m_driveTrain.setState(State.HIGH_GEAR_OPEN_LOOP);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // if(fromAuto){
    //   m_arm.setTargetState(m_arm.getLastState());
    //   m_wrist.setTargetState(m_wrist.getLastState());
    // }
    // else{
      m_arm.setTargetState(Arm.State.STOWED);
      m_wrist.setTargetState(Wrist.State.STOWED);
    // }
    fromAuto = false;
    // m_eye.look();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // m_eye.look();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

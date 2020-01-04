/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  XboxController driver;
  XboxController operator;
  private static OI instance;

  private OI() {
    driver = new XboxController(0);
    operator = new XboxController(1);
  }

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private static final double DEADBAND_RADIUS = 0.15;
  private static double deadband(double jsValue) {
    if (Math.abs(jsValue) < DEADBAND_RADIUS){
      return 0;
    }
    jsValue *= 1.0 / (1.0-DEADBAND_RADIUS);
    return jsValue;
  }

  public double getXLeftDriver() {
    return deadband(driver.getX(Hand.kLeft));
  }
  public double getXLeftOperator() {
    return deadband(operator.getX(Hand.kLeft));
  }

  public double getYLeftDriver(){
    return deadband(driver.getY(Hand.kLeft));
  }
  public double getYLeftOperator(){
    return deadband(operator.getY(Hand.kLeft));
  }

  public double getXRightDriver() {
    return deadband(driver.getX(Hand.kRight));
  }
  public double getXRightOperator() {
    return deadband(operator.getX(Hand.kRight));
  }

  public double getYRightDriver() {
    return deadband(driver.getY(Hand.kRight));
  }
  public double getYRightOperator() {
    return deadband(operator.getY(Hand.kRight));
  }

  public boolean getAButtonDriver() {
    return driver.getAButton();
  }
  public boolean getAButtonOperator() {
    return operator.getAButton();
  }

  public boolean getBButtonDriver() {
    return driver.getBButton();
  }
  public boolean getBButtonOperator() {
    return operator.getBButton();
  }

  public boolean getXButtonDriver() {
    return driver.getXButton();
  }
  public boolean getXButtonOperator() {
    return operator.getXButton();
  }

  public boolean getYButtonDriver() {
    return driver.getYButton();
  }
  public boolean getYButtonOperator() {
    return driver.getYButton();
  }

  public boolean getLeftBumperDriver() {
    return driver.getBumper(Hand.kLeft);
  }
  public boolean getLeftBumperOperator() {
    return operator.getBumper(Hand.kLeft);
  }

  public boolean getRightBumperDriver() {
    return driver.getBumper(Hand.kRight);
  }
  public boolean getRightBumperOperator() {
    return operator.getBumper(Hand.kRight);
  }

  public boolean getBackButtonDriver() {
    return driver.getBackButton();
  }
  public boolean getBackButtonOperator() {
    return operator.getBackButton();
  }

  public boolean getStartButtonDriver(){
    return driver.getStartButton();
  }
  public boolean getStartButtonOperator(){
    return operator.getStartButton();
  }

  public boolean getLeftStickButtonDriver(){
    return driver.getStickButton(Hand.kLeft);
  }
  public boolean getLeftStickButtonOperator(){
    return operator.getStickButton(Hand.kLeft);
  }

  public boolean getRightStickButtonDriver(){
    return driver.getStickButton(Hand.kRight);
  }  
  public boolean getRightStickButtonOperator(){
    return operator.getStickButton(Hand.kRight);
  }

  public double getLeftTriggerDriver(){
    return driver.getTriggerAxis(Hand.kLeft);
  }
  public double getLeftTriggerOperator(){
    return operator.getTriggerAxis(Hand.kLeft);
  }

  public double getRightTriggerDriver() {
    return driver.getTriggerAxis(Hand.kRight);
  }
  public double getRightTriggerOperator() {
    return operator.getTriggerAxis(Hand.kRight);
  }

  public boolean getDPadLeftDriver(){
    return driver.getPOV() == 270;
  }
  public boolean getDPadLeftOperator(){
    return operator.getPOV() == 270;
  }

  public boolean getDPadRightDriver() {
    return driver.getPOV() == 90;
  }
  public boolean getDPadRightOperator() {
    return operator.getPOV() == 90;
  }

  public boolean getDPadUpDriver() {
    return driver.getPOV() == 0;
  }
  public boolean getDPadUpOperator() {
    return operator.getPOV() == 0;
  }

  public boolean getDPadDownDriver() {
    return driver.getPOV() == 180;
  }
  public boolean getDPadDownOperator() {
    return operator.getPOV() == 180;
  }


  public boolean getAButtonPressedDriver() {
    return (driver.getAButtonPressed());
  }
  public boolean getAButtonPressedOperator() {
    return (operator.getAButtonPressed());
  }


  public boolean getBButtonPressedDriver() {
    return (driver.getBButtonPressed());
  }
  public boolean getBButtonPressedOperator() {
    return (operator.getBButtonPressed());
  }

  public boolean getXButtonPressedDriver() {
    return (driver.getXButtonPressed());
  }
  public boolean getXButtonPressedOperator() {
    return (operator.getXButtonPressed());
  }

  public boolean getYButtonPressedDriver() {
    return (driver.getYButtonPressed());
  }
  public boolean getYButtonPressedOperator() {
    return (operator.getYButtonPressed());
  }

  public boolean getLeftBumperPressedDriver() {
    return (driver.getBumperPressed(Hand.kLeft));
  }
  public boolean getLeftBumperPressedOperator() {
    return (operator.getBumperPressed(Hand.kLeft));
  }

  public boolean getRightBumperPressedDriver() {
    return (driver.getBumperPressed(Hand.kRight));
  }
  public boolean getRightBumperPressedOperator() {
    return (operator.getBumperPressed(Hand.kRight));
  }

  public boolean getBackButtonPressedDriver() {
    return (driver.getBackButtonPressed());
  }
  public boolean getBackButtonPressedOperator() {
    return (operator.getBackButtonPressed());
  }

  public boolean getStartButtonPressedDriver() {
    return (driver.getStartButtonPressed());
  }
  public boolean getStartButtonPressedOperator() {
    return (operator.getStartButtonPressed());
  }

  public boolean getLeftStickButtonPressedDriver() {
    return (driver.getStickButtonPressed(Hand.kLeft));
  }
  public boolean getLeftStickButtonPressedOperator() {
    return (operator.getStickButtonPressed(Hand.kLeft));
  }

  public boolean getRightStickButtonPressedDriver() {
    return (driver.getStickButtonPressed(Hand.kRight));
  }
  public boolean getRightStickButtonPressedOperator() {
    return (operator.getStickButtonPressed(Hand.kRight));
  }

  public void rumbleDriver(double power) {
    driver.setRumble(Joystick.RumbleType.kLeftRumble, power);
    driver.setRumble(Joystick.RumbleType.kRightRumble, power);
  }

  public void rumbleOperator(double power) {
    operator.setRumble(Joystick.RumbleType.kLeftRumble, power);
    operator.setRumble(Joystick.RumbleType.kRightRumble, power);
  }
}
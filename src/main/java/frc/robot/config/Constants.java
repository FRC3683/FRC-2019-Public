/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.config;

import frc.robot.utils.MathUtils;

/**
 * Add your docs here.
 */
public class Constants {
    public static final double driveTicksPerInchLeft = 490.896; //461.695354; //442.74375;
    public static final double driveTicksPerInchRight = 490.896; //461.695354; //453.7520833;

    public static final double driveStraightKP = 0.2;
    public static final double driveStraightKI = 0.0;
    public static final double driveStraightKD = 0.008;
    public static final double driveStraightTolerance = 1.0;

    public static final double driveTurnKP = 0.05;//1.25, 0.3
    public static final double driveTurnKI = 0.00003;
    public static final double driveTurnKD = 0.1;//0.155, 0.038
    public static final double driveTurnTolerance = 1.5;//2

    public static final double LENGTH_FROM_BACK=36;

    public static final double getTicksPerInch(){
        return MathUtils.average(driveTicksPerInchLeft, driveTicksPerInchRight);
    }
    //Sensor models
    public static final double wristPotRawOffset = 0.00;
    public static final double wristPotRawMax = 1.00;
    public static final double wristPotAngleOffset = 304-3;
    public static final double wristPotValueToAngle = -600;
    public static final double armEncoderTicksPerDegree = 33.02;
    public static final double armHomeOffset = -54;

    //PID for wrist
    public static final double wristkP = 0.029;
    public static final double wristkI = 0.0001;
    public static final double wristkD = 0.00000;
    public static final double wristkTol = 2;
    //PID for the arm
    public static final double armKP = 0.19;
    public static final double armKI = 0.00035;
    public static final double armKD = 0.00;
    public static final double armTol = 2;

    //arm and wrist motion stagger thresholds
    public static final double armStaggerThreshold = 60;
    public static final double armStaggerThresholdMid = 30;
    public static final double wristStaggerThreshold = 140;
    public static final double wristStaggerThresholdExitClimb = 60;

    //wrist position constants, degrees
    public static final double wristStowedPos = 154;
    public static final double wristHoldingGamePiece = 154;
    public static final double wristIntakeBallFloor = 30;
    public static final double wristIntakeBallHP = 75;
    public static final double wristScoreBall1 = 100; //100
    public static final double wristScoreBall2 = -10;
    public static final double wristScoreBall3 = 15;
    public static final double wristScoreBallCargo = -66;
    public static final double wristIntakeDiscPos = 139;
    public static final double wristScoreDisc1 = 140;
    public static final double wristScoreDisc2 = 88;
    public static final double wristScoreDisc3 = 32;
    public static final double wristClimbSetup = -84;
    public static final double wristClimb = -60;
    
    //Arm position constants, degrees
    public static final double armStowedPos = -54; //-53
    public static final double armHoldingGamePiece = -53;
    public static final double armIntakeBallFloor = -40;
    public static final double armIntakeBallHP = -15;
    public static final double armScoreBall1 = -41; //-38
    public static final double armScoreBall2 = 30;
    public static final double armScoreBall3 = 60;
    public static final double armScoreBallCargo = 29;
    public static final double armIntakeDiscPos = -52;
    public static final double armScoreDisc1 = -51;
    public static final double armScoreDisc2 = -1;
    public static final double armScoreDisc3 = 51;
    public static final double armUpperLimit = 60;
    public static final double armThrottleThreshold = -25;
    public static final double armClimbSetup = -13;
    public static final double armClimb = -43.8;

    //delay between disc fingers opening and disc being punched out, in seconds
    public static final double discShotDelay = 0.15;

    public static final String SuperStructureTabName = "Super Structure";
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoCommandGroups.pointToPointSegments;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.DriveStraightToTarget;
import frc.robot.commands.HandToState;
import frc.robot.commands.Wait;
import frc.robot.commands.WristToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Hand.HandState;

public class Seg_H1C_CFX extends CommandGroup {
  /**
   * Segment from center on hab1 to front of cargo
   */
  public Seg_H1C_CFX() {
    addSequential(new DriveStraightToTarget(112.00));
  }
}

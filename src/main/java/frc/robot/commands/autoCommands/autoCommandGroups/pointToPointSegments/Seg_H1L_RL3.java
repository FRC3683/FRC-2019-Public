/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoCommandGroups.pointToPointSegments;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveStraightToTarget;
import frc.robot.commands.TurnToAngle;

public class Seg_H1L_RL3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_H1L_RL3() {
    addSequential(new DriveStraightToTarget(223.83));
    addSequential(new TurnToAngle(90.0));
    addSequential(new DriveStraightToTarget(72.35));
    addSequential(new TurnToAngle(118.75));
   // addSequential(new DriveStraightToTarget(//*Constants.getTicksPerInch()));
  }
}

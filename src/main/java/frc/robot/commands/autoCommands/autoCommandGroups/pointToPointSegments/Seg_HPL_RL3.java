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

public class Seg_HPL_RL3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_HPL_RL3() {
    addSequential(new DriveStraightToTarget(-64.25));
    addSequential(new TurnToAngle(155.95));
    addSequential(new DriveStraightToTarget(211.16));
    addSequential(new TurnToAngle(114.05));
    addSequential(new DriveStraightToTarget(80.3));
    addSequential(new TurnToAngle(59.375));
    addSequential(new DriveStraightToTarget(21.05));

  }
}

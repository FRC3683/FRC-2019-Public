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

public class Seg_HPL_RL1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_HPL_RL1() {
    addSequential(new TurnToAngle(-185.08));
    addSequential(new DriveStraightToTarget(170.55));
    addSequential(new TurnToAngle(69.17));
    addSequential(new DriveStraightToTarget(19.25));
  }
}

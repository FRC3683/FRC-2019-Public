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

public class Seg_RL3_HPL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_RL3_HPL() {
    addSequential(new DriveStraightToTarget(-21.05));
    addSequential(new TurnToAngle(120.625));
    addSequential(new DriveStraightToTarget(80.3));
    addSequential(new TurnToAngle(-114.05));
    addSequential(new DriveStraightToTarget(211.16));
    addSequential(new TurnToAngle(24.05));
    addSequential(new DriveStraightToTarget(64.25));



  }
}

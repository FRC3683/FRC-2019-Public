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

public class Seg_CFL_HPL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_CFL_HPL() {
    addSequential(new DriveStraightToTarget(-30.13));
    addSequential(new TurnToAngle(-41.46));
    addSequential(new DriveStraightToTarget(203.22));
    addSequential((new TurnToAngle(138.54))); 
    addSequential(new DriveStraightToTarget(51.89));
  }
}

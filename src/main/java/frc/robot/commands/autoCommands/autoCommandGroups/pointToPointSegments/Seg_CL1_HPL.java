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

public class Seg_CL1_HPL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_CL1_HPL() {
    addSequential(new DriveStraightToTarget(-19.63));
    addSequential(new TurnToAngle(-113.22));
    addSequential(new DriveStraightToTarget(210.62));
    addSequential((new TurnToAngle(-156.78))); 
    addSequential(new DriveStraightToTarget(27.75));
  }
}

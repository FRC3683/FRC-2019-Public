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

public class Seg_CL3_HPL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Seg_CL3_HPL() {
    addSequential(new DriveStraightToTarget(-19.63));
    addSequential(new TurnToAngle(-110.81));
    addSequential(new DriveStraightToTarget(238.74));
    addSequential((new TurnToAngle(-159.19))); 
    addSequential(new DriveStraightToTarget(27.75));
  }
}

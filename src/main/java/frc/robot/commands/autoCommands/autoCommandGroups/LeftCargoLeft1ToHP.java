/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveStraightToTarget;
import frc.robot.commands.TurnToAngle;

public class LeftCargoLeft1ToHP extends CommandGroup {
  
  public LeftCargoLeft1ToHP() {
    addSequential(new LeftCargoLeft1());
    addSequential(new DriveStraightToTarget(-21.13));
    addSequential(new TurnToAngle(23.59+90.0));
    addSequential(new DriveStraightToTarget(211.22));
    addSequential(new TurnToAngle(-23.59));
    addSequential(new DriveStraightToTarget(45));
  
  }
}

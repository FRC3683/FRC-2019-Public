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

public class CargoLeft1ToHP extends CommandGroup {
  public CargoLeft1ToHP() {
  addSequential(new DriveStraightToTarget(-35.625));
  addSequential(new TurnToAngle(-90));
  addSequential(new DriveStraightToTarget(-40.55));
  addSequential(new TurnToAngle(-30.639));
  addSequential(new DriveStraightToTarget(137.48));
  addSequential(new TurnToAngle(30.639));
  addSequential(new DriveStraightToTarget(77.28));
  }
}

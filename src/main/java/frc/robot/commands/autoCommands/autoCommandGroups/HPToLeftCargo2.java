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

public class HPToLeftCargo2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HPToLeftCargo2() {
   addSequential(new DriveStraightToTarget(-45));
   addSequential(new DriveStraightToTarget(-231.31));
   addSequential(new TurnToAngle(111.41));
   addSequential(new DriveStraightToTarget(21.13));
  }
}

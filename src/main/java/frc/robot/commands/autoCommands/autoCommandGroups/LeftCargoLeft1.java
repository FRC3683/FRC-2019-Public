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

public class LeftCargoLeft1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftCargoLeft1() {
   
    addSequential(new DriveStraightToTarget(193.55, 0));// drive off the ramp to cargo ship target1
    addSequential(new TurnToAngle(90));//turn to score
    addSequential(new DriveStraightToTarget(21.13, 90));// drive to scoring target
      
   
  }
}

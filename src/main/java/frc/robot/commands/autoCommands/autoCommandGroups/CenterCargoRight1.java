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

public class CenterCargoRight1 extends CommandGroup {
 
  public CenterCargoRight1() {
    

    addSequential(new DriveStraightToTarget(60.0));// drive off ramp distance 
    addSequential(new TurnToAngle(-34.178));// angle to right
    addSequential(new DriveStraightToTarget(113.926));// drive to left
    addSequential(new TurnToAngle(34.178));//angle backforward
    addSequential(new DriveStraightToTarget(40.55));// drive straight
    addSequential(new TurnToAngle(90.0));//turn to score
    addSequential(new DriveStraightToTarget(30.0));// drive to scoring target
    
  }
}

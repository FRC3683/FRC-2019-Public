/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hand.HandState;
import frc.robot.subsystems.Wrist;

public class TestAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestAuto() {
    addSequential(new TurnToAngle(45));
  }
}

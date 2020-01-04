/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands.autoCommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ArmWristTeleopControl;
import frc.robot.commands.Drive;
import frc.robot.commands.HandTeleOpControl;
import frc.robot.commands.SpiderTeleOpControl;
import frc.robot.commands.Drive.DriveMode;

public class SandstormTeleop extends CommandGroup {
	/**
	 * Add your docs here.
	 */
	public SandstormTeleop() {
		addParallel(new Drive(DriveMode.ARCADE_TWO_STICK));
		addParallel(new ArmWristTeleopControl());
		addParallel(new HandTeleOpControl());
		addParallel(new SpiderTeleOpControl());
	}
}

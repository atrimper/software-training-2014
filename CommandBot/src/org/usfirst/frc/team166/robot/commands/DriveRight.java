package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveRight extends CommandGroup {
	public DriveRight() {
		addSequential(new StrafeDirection(1,.5));//not using power right now, instead using preferences.
	}

}

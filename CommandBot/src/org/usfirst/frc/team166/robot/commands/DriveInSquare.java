package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveInSquare extends CommandGroup {
	public DriveInSquare() {
		addSequential(new DriveDirection(0), 2.0);
		addSequential(new DriveDirection(90), 2.0);
		addSequential(new DriveDirection(180), 2.0);
		addSequential(new DriveDirection(270), 2.0);
	}

}

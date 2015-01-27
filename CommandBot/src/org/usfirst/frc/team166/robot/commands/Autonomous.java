package org.usfirst.frc.team166.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Autonomous extends CommandGroup {
	public Autonomous() {
		addSequential(new DriveToStep());
		//addSequential(new DriveDirection(180), .025);
		addSequential(new CenterOnStep(), 3.0);
		addSequential(new DriveDirection(0), .5);
	}

}

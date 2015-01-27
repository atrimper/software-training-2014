package org.usfirst.frc.team166.robot.commands;

import org.usfirst.frc.team166.robot.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CenterOnStep extends Command {

	public CenterOnStep() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.centerOnStep(); //Add Gyro Later
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		//return Robot.driveTrain.isCentered();
		return false;
		
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}

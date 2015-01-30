package org.usfirst.frc.team166.robot.commands;

import org.usfirst.frc.team166.robot.Robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToStep extends Command {

	public DriveToStep() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.printPDPAverageCurrent();
		Robot.driveTrain.driveToStep(); //Add Gyro Later
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		//return false;
		return (Robot.driveTrain.getPDPAverageCurrent() > Preferences.getInstance().getDouble("currentCutOff", 1));
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.resetIntegral();
		Robot.driveTrain.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}

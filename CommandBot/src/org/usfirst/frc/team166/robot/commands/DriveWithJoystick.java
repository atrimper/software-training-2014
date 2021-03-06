package org.usfirst.frc.team166.robot.commands;

import org.usfirst.frc.team166.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithJoystick extends Command {

	public DriveWithJoystick() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.driveTrain.toggleFPSDrive()){
			Robot.driveTrain.xboxDrive(Robot.oi.getXbox());
		}
		else {
			Robot.driveTrain.mecanumDrive(Robot.oi.getJoystick());
		}
		Robot.driveTrain.mecanumDrive(Robot.oi.getJoystick());
		Robot.driveTrain.putAccelValues();
		Robot.driveTrain.getGyro();
		Robot.driveTrain.printEncoderValues();
		Robot.driveTrain.printDistances();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}

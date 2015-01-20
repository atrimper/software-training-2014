package org.usfirst.frc.team166.robot.commands;

import org.usfirst.frc.team166.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StrafeDirection extends Command {
	private int strafeDirection;
	private double strafePower;

	public StrafeDirection(int direction, double power) {
		this.strafeDirection = direction;
		this.strafePower = power;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.strafeWithGyro(strafeDirection,strafePower);
		Robot.driveTrain.printEncoderValues();
		
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

package org.usfirst.frc.team166.robot.subsystems;

import org.usfirst.frc.team166.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;
/**
 *
 */
public class DriveTrain extends Subsystem {
	RobotDrive robotDrive;
	public Gyro gyro;
	PowerDistributionPanel pdp;
	AnalogInput rangefinder;
	BuiltInAccelerometer accel;

	public DriveTrain() {

		// Channels for the wheels

		final int frontLeftChannel = 2;
		final int rearLeftChannel = 3;
		final int frontRightChannel = 1;
		final int rearRightChannel = 0;

		// The channel on the driver station that the joystick is connected to
		final int joystickChannel = 0;
		// rangefinder stuffs
		rangefinder = new AnalogInput(3);
		accel = new BuiltInAccelerometer();
		gyro = new Gyro(0);
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel,
				frontRightChannel, rearRightChannel);
		robotDrive.setExpiration(0.1);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

	}
	
	public void mecanumDrive(Joystick stick) {
		if((Math.abs(stick.getX()) > .1) || (Math.abs(stick.getY()) > .1) || (Math.abs(stick.getRawAxis(3)) > .1)){
			robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(),
					stick.getRawAxis(3), 0);
		}
	}

	public void driveForward() {
		robotDrive.mecanumDrive_Cartesian(0.0, .25, 0.0, 0);
	}

	public void driveAngle(int angle) {
		robotDrive
				.mecanumDrive_Polar(
						Preferences.getInstance().getDouble("OldManSpeed", 0),
						angle, 0);
	}

	public void getDistance() {
		SmartDashboard.putNumber("Distance", rangefinder.getAverageVoltage());
	}

	public void initGyro() {
		gyro.setSensitivity(.0125);

	}
	public void getGyro(){
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
	}

	public void putAccelValues() {
		SmartDashboard.putNumber("X:", accel.getX());
		SmartDashboard.putNumber("Y:", accel.getY());
		SmartDashboard.putNumber("Z:", accel.getZ());
	}
	public void getDistanceTraveled(){
		//private double XDistance = accel.getX()
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}
}

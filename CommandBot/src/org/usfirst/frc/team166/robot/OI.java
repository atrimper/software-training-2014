package org.usfirst.frc.team166.robot;

import org.usfirst.frc.team166.robot.commands.Autonomous;
import org.usfirst.frc.team166.robot.commands.CancelDriveCommand;
import org.usfirst.frc.team166.robot.commands.DriveInSquare;
import org.usfirst.frc.team166.robot.commands.DriveLeft;
import org.usfirst.frc.team166.robot.commands.DriveRight;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private Joystick threeAxisJoy = new Joystick(0);
	private Joystick xbox = new Joystick (1);

	public OI() {
		JoystickButton xboxButtonA = new JoystickButton(xbox, 1);
		JoystickButton xboxButtonB = new JoystickButton(xbox, 2);
		JoystickButton xboxButtonX = new JoystickButton(xbox, 3);
		JoystickButton button1 = new JoystickButton(threeAxisJoy, 1);
		JoystickButton button2 = new JoystickButton(threeAxisJoy, 2);
		JoystickButton button3 = new JoystickButton(threeAxisJoy, 3);
		JoystickButton button4 = new JoystickButton(threeAxisJoy, 4);
		xboxButtonA.whenPressed(new CancelDriveCommand());
		xboxButtonB.whileHeld(new DriveRight());
		xboxButtonX.whileHeld(new DriveLeft());
		button1.whenPressed(new DriveInSquare());
		button2.whenPressed(new CancelDriveCommand());
		button3.whileHeld(new DriveLeft());
		button4.whenPressed(new Autonomous());
	}
	public Joystick getXbox() {
		return xbox;
	}
	public Joystick getJoystick() {
		return threeAxisJoy;
	}
	// add public OI for dashboard buttons
}

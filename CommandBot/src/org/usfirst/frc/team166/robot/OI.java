package org.usfirst.frc.team166.robot;

import org.usfirst.frc.team166.robot.commands.CancelDriveCommand;
import org.usfirst.frc.team166.robot.commands.DriveInSquare;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private Joystick threeAxisJoy = new Joystick(0);

	public OI() {
		JoystickButton button1 = new JoystickButton(threeAxisJoy, 1);
		JoystickButton button2 = new JoystickButton(threeAxisJoy, 2);
		button1.whenPressed(new DriveInSquare());
		button2.whenPressed(new CancelDriveCommand());
	}

	public Joystick getJoystick() {
		return threeAxisJoy;
	}
	// add public OI for dashboard buttons
}

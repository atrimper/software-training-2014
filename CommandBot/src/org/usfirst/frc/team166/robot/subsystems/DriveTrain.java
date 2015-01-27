package org.usfirst.frc.team166.robot.subsystems;

import org.usfirst.frc.team166.robot.PIDSpeedController;
import org.usfirst.frc.team166.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

import java.lang.Math;
/**
 *
 */
public class DriveTrain extends Subsystem {
	//PIDController encoderPID;
	RobotDrive robotDrive;
	public Gyro gyro;
	PowerDistributionPanel pdp;
	AnalogInput leftUS;
	AnalogInput rightUS;
	AnalogInput frontUS;
	AnalogInput analogTester;
	BuiltInAccelerometer accel;
	double gyroOffset;
	//wheel names
	PIDSpeedController frontLeftPID;
	PIDSpeedController frontRightPID;
	PIDSpeedController rearLeftPID;
	PIDSpeedController rearRightPID;
	Talon frontLeftTalon;
	Talon rearLeftTalon;
	Talon frontRightTalon;
	Talon rearRightTalon;
	Encoder frontLeftEncoder;
	Encoder frontRightEncoder;
	Encoder rearLeftEncoder;
	Encoder rearRightEncoder;
	boolean isUserButtonFPS = false;
	double derp;
	double USDistance;
	double averageCurrent;
	double joystickScaler;
	double centerOffsetDistance;
	int driveMode = 0;
	public DriveTrain() {


		// The Talon on the driver station that the joystick is connected to
		final int joystickChannel = 0;
		pdp = new PowerDistributionPanel();
		//SPEED CONTROLLERS
		rearLeftTalon = new Talon(0);
		rearRightTalon = new Talon(1);
		frontLeftTalon = new Talon(2);
		frontRightTalon = new Talon(3);
		
		//ENCODERS
		rearLeftEncoder = new Encoder(0,1);
		rearRightEncoder = new Encoder(2,3);
		frontLeftEncoder = new Encoder(4,5);
		frontRightEncoder = new Encoder(6,7);
		
		//ENCODER MATH
		frontLeftEncoder.setDistancePerPulse(((6*Math.PI)/1024) / 183);
		frontLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		frontRightEncoder.setDistancePerPulse(((6*Math.PI)/1024) / 183);
		frontRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearLeftEncoder.setDistancePerPulse(((6*Math.PI)/1024) / 183);
		rearLeftEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		rearRightEncoder.setDistancePerPulse(((6*Math.PI)/1024) / 183);
		rearRightEncoder.setPIDSourceParameter(PIDSourceParameter.kRate);
		
		//PID SPEED CONTROLLERS
		frontLeftPID = new PIDSpeedController(frontLeftEncoder,frontLeftTalon,"frontLeft");
		frontRightPID = new PIDSpeedController(frontRightEncoder,frontRightTalon,"frontRight");
		rearLeftPID = new PIDSpeedController(rearLeftEncoder,rearLeftTalon,"rearLeft");
		rearRightPID = new PIDSpeedController(rearRightEncoder,rearRightTalon,"rearRight");
		
		//OTHER SENSORS
		frontUS = new AnalogInput(1);
		rightUS = new AnalogInput(2);
		leftUS = new AnalogInput(3);
		accel = new BuiltInAccelerometer();
		gyro = new Gyro(0);
		//DRIVE DECLARATION
		robotDrive = new RobotDrive(frontLeftPID, rearLeftPID,
				frontRightPID, rearRightPID);
		robotDrive.setExpiration(0.1);
		//MOTOR INVERSIONS
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
	}
	
	public void mecanumDrive(Joystick stick) {
		printPDPAverageCurrent();
		joystickScaler = Preferences.getInstance().getDouble("joystickScaler", 1);
		if((Math.abs(stick.getX()) > .1) || (Math.abs(stick.getY()) > .1) || (Math.abs(stick.getRawAxis(3)) > .1)){
			if(Math.abs(stick.getRawAxis(3)) > .1){
				robotDrive.mecanumDrive_Cartesian(stick.getX() * joystickScaler, stick.getY() * joystickScaler,stick.getRawAxis(3) * joystickScaler, 0);
//				robotDrive.mecanumDrive_Cartesian((stick.getX()/Math.abs(stick.getX())) * Math.min(Math.abs(stick.getX()),.85),
//						(stick.getY()/Math.abs(stick.getY())) * Math.min(Math.abs(stick.getY()),.85),
//						(stick.getRawAxis(3)/Math.abs(stick.getRawAxis(3))) * Math.min(Math.abs(stick.getRawAxis(3)),.85), 0);
				driveMode = 0; //using joy for all
			}
			else{
				if(driveMode == 0){ //first time in loop?
					gyro.reset(); 
				}
				gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .033333333));
				driveMode = 1; //using gyro for rotation
				if(Math.abs(gyroOffset) > 1){
					gyroOffset = (Math.abs(gyroOffset)/gyroOffset);
				}
				robotDrive.mecanumDrive_Cartesian(stick.getX()*joystickScaler,stick.getY()*joystickScaler,-gyroOffset,0);
			}
			SmartDashboard.putNumber("DriveMode", driveMode);
			
		}
		else
		{
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			gyro.reset();
		}
	}
	public void xboxDrive(Joystick xbox) {
			robotDrive.mecanumDrive_Cartesian(xbox.getRawAxis(0), xbox.getRawAxis(1), xbox.getRawAxis(4), 0 );
	}
	public void driveForward() {
		robotDrive.mecanumDrive_Cartesian(0.0, .25, 0.0, 0);
	}
	public void strafeWithGyro(int direction,double power){ // -1 is left, 1 is right //Not using power right now for testing
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		if(Math.abs(gyroOffset) > 1){
			gyroOffset = (Math.abs(gyroOffset)/gyroOffset);
		}
		robotDrive.mecanumDrive_Cartesian(Preferences.getInstance().getDouble("StrafePower", 0) * direction,0,-gyroOffset,0);
	}
	public void resetIntegral(){
		frontLeftPID.reset();
		frontRightPID.reset();
		rearLeftPID.reset();
		rearRightPID.reset();
	}
	public void driveAngle(int angle) {
		//robotDrive.mecanumDrive_Polar(Preferences.getInstance().getDouble("OldManSpeed", 0),angle, 0);
		robotDrive.mecanumDrive_Cartesian(0,Preferences.getInstance().getDouble("OldManSpeed", 0),0,0);
	}
	public void driveToStep(){
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		if(Math.abs(gyroOffset) > 1){
			gyroOffset = (Math.abs(gyroOffset)/gyroOffset);
		}
		robotDrive.mecanumDrive_Cartesian(0 , Preferences.getInstance().getDouble("fastAutoSpeed", -.25), -gyroOffset, 0);
	}
	public void centerOnStep(){
		gyroOffset = (getGyro() * Preferences.getInstance().getDouble("GyroStrafeConstant", .01111111111));
		if(Math.abs(gyroOffset) > 1){
			gyroOffset = (Math.abs(gyroOffset)/gyroOffset);
		}
		centerOffsetDistance = getRightUS() - getLeftUS();
		robotDrive.mecanumDrive_Cartesian(centerOffsetDistance / Preferences.getInstance().getDouble("USCenterDistanceConstant", 27.5) , 0, -gyroOffset, 0);
	}
	public boolean isCentered(){
		return (Math.abs(getRightUS() - getLeftUS()) < 1);
	}
	public void stopMotors(){
		robotDrive.mecanumDrive_Cartesian(0,0,0,0);
	}
	public void printDistances(){
		USDistance = rightUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("rightUS", USDistance);
		USDistance = leftUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("leftUS", USDistance);
		USDistance = frontUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("frontUS", USDistance);
	}
	public double getRightUS() {
		USDistance = rightUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("rightUS", USDistance);
		return USDistance;
	}
	public double getLeftUS() {
		USDistance = leftUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("leftUS", USDistance);
		return USDistance;
	}
	public double getFrontUS() {
		USDistance = frontUS.getAverageVoltage() * Preferences.getInstance().getDouble("USConstant", 102.0408163265306);
		SmartDashboard.putNumber("frontUS", USDistance);
		return USDistance;
	}
	public void printAnalogValue() {
		SmartDashboard.putNumber("AnalogTest", analogTester.getAverageVoltage());
	}
	public boolean toggleFPSDrive() {
		isUserButtonFPS ^= Utility.getUserButton();
		return isUserButtonFPS;
	}
	public void initGyro() {
		gyro.initGyro();
		gyro.setSensitivity(.0125);
		
	}
	public double getGyro(){
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
		return gyro.getAngle();
	}
	
	public void printEncoderValues(){
		double inchesTraveled = (frontLeftEncoder.getDistance() / 1024) * (2 * Preferences.getInstance().getDouble("WheelRadius", 3) * Math.PI);
		SmartDashboard.putNumber("Inches Traveled", inchesTraveled);
		SmartDashboard.putNumber("Encoder Speed FL", frontLeftEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed FR", frontRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RR", rearRightEncoder.getRate());
		SmartDashboard.putNumber("Encoder Speed RL", rearLeftEncoder.getRate());
	}
	public void putAccelValues() {
		SmartDashboard.putNumber("X:", accel.getX());
		SmartDashboard.putNumber("Y:", accel.getY());
		SmartDashboard.putNumber("Z:", accel.getZ());
	}
	public void getDistanceTraveled(){
//		private double XDistance = accel.getX()
	}
	public double getPDPAverageCurrent(){
		//averageCurrent = ((pdp.getCurrent(0) + pdp.getCurrent(1) + pdp.getCurrent(2) +pdp.getCurrent(3)) / 4);
		averageCurrent = pdp.getCurrent(0);
		
		SmartDashboard.putNumber("Average Current", averageCurrent);
		return averageCurrent;
	}
	public void printPDPAverageCurrent(){
		//averageCurrent = ((pdp.getCurrent(0) + pdp.getCurrent(1) + pdp.getCurrent(2) +pdp.getCurrent(3)) / 4);
		averageCurrent = pdp.getCurrent(0);
		SmartDashboard.putNumber("Average Current", averageCurrent);
	}
	public void setPIDConstants(){
		frontLeftPID.setConstants();
		frontRightPID.setConstants();
		rearLeftPID.setConstants();
		rearRightPID.setConstants();
	}

	public void initDefaultCommand() {
//		 Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}
}

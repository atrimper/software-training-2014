package org.usfirst.frc.team166.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXL345_I2C;

public class Navigation {
	Timer timer;
	Timer timer2;
	ADXL345_I2C threeAxisAccel;
	AnalogAccelerometer accelX;
	AnalogAccelerometer accelY;
	DriverStation DS;
	BuiltInAccelerometer accel;
	Gyro gyro;
	double xAccelG = 0;
	double yAccelG = 0;
	double zAccelG = 0;
	double xAccelF = 0;
	double yAccelF = 0;
	double zAccelF = 0;
	double xVel = 0;
	double yVel = 0;
	double zVel = 0;
	double xPos = 0;
	double yPos = 0;
	double zPos = 0;
	double biasXTotal = 0;
	double biasYTotal = 0;
	double biasZTotal = 0;
	double biasX = 0;
	double biasY = 0;
	double biasZ = 0;
	double endTime = 0;
	double startTime = 0;
	double conversionGfConstant = 32.174049;
	double sampleCount = 0;
	double lastCycle = 0;
	double cycleTime = 0;
	double currentTime = 0;
	int chops = 10000;
	double headingAngle;
	
	public Navigation(){ //constructor
		threeAxisAccel = new ADXL345_I2C(I2C.Port.kOnboard,Accelerometer.Range.k4G);
		timer = new Timer();
		timer2 = new Timer();
		accelX = new AnalogAccelerometer(1);
		accelY = new AnalogAccelerometer(2);
		DS = DriverStation.getInstance();
		accel = new BuiltInAccelerometer();
		//gyro = new Gyro(0);
	}
	public void reset(){
		xAccelG = 0;
		yAccelG = 0;
		zAccelG = 0;
		xAccelF = 0;
		yAccelF = 0;
		zAccelF = 0;
		xVel = 0;
		yVel = 0;
		zVel = 0;
		xPos = 0;
		yPos = 0;
		zPos = 0;
		endTime = 0;
		startTime = 0;
	}
	public void initSensors(){
		accel.setRange(Accelerometer.Range.k8G);
		accelX.setSensitivity(1.0);
		accelY.setSensitivity(1.0);
		accelX.setZero(0.0);
		accelY.setZero(0.0);
		startTime = Timer.getFPGATimestamp();
		while(Timer.getFPGATimestamp() < startTime + 5){
			if(Timer.getFPGATimestamp() >= endTime + .001){
				biasXTotal += threeAxisAccel.getX();
				biasYTotal += threeAxisAccel.getY();
				biasZTotal += threeAxisAccel.getZ();
				endTime = Timer.getFPGATimestamp();
				sampleCount += 1;
			}
		}
		
		biasX = (biasXTotal / sampleCount) * conversionGfConstant;
		biasY = (biasYTotal / sampleCount) * conversionGfConstant;
		biasZ = (biasZTotal / sampleCount) * conversionGfConstant;
		SmartDashboard.putNumber("SampleCount:", sampleCount);
		SmartDashboard.putNumber("X Bias:", biasX);
		SmartDashboard.putNumber("Y Bias:", biasY);
		SmartDashboard.putNumber("Z Bias:", biasZ);
	}
	
	
	public void takeReadings(){
		lastCycle = Timer.getFPGATimestamp();
		while(DS.isEnabled()){
			if(Timer.getFPGATimestamp() >= endTime + .001){
				currentTime = Timer.getFPGATimestamp();
				cycleTime = currentTime - lastCycle;
				//Take Readings
				
				xAccelG = threeAxisAccel.getX();
				yAccelG = threeAxisAccel.getY();
				zAccelG = threeAxisAccel.getZ();
				SmartDashboard.putNumber("X:", (threeAxisAccel.getX() * conversionGfConstant) - biasX);
				SmartDashboard.putNumber("Y:", (threeAxisAccel.getY() * conversionGfConstant)-  biasY);
				SmartDashboard.putNumber("Z:", (threeAxisAccel.getZ() * conversionGfConstant)-  biasZ);
				//headingAngle = gyro.getAngle();
				
				integrate();
				endTime = Timer.getFPGATimestamp();
				lastCycle = currentTime;
			}
		}
		
	}
	public void integrate(){
		//convert g's to f/s^2
		xAccelF = xAccelG * conversionGfConstant;
		yAccelF = yAccelG * conversionGfConstant;
		zAccelF = zAccelG * conversionGfConstant;
		
		//integrate
		for(int i = 0;i <= chops; i++)
		{
			xVel += (xAccelF - biasX) * (cycleTime/chops);
		
			yVel += (yAccelF - biasY) * (cycleTime/chops);
			
			zVel += (zAccelF- biasZ) * (cycleTime/chops);
			
			xPos += xVel * (cycleTime/chops);
			
			yPos += yVel * (cycleTime/chops);
			
			zPos += zVel * (cycleTime/chops);
		}
			
		SmartDashboard.putNumber("Cycle Time", cycleTime);
		SmartDashboard.putNumber("X Vel:", xVel);
		SmartDashboard.putNumber("Y Vel:", yVel);
		SmartDashboard.putNumber("Z Vel:", zVel);
		SmartDashboard.putNumber("X Pos:", xPos);
		SmartDashboard.putNumber("Y Pos:", yPos);
		SmartDashboard.putNumber("Z Pos:", zPos);
		

	}
}

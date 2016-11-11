// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc199.OffseasonBot2016.subsystems;

import org.usfirst.frc199.OffseasonBot2016.DashboardInterface;
import org.usfirst.frc199.OffseasonBot2016.PID;
import org.usfirst.frc199.OffseasonBot2016.RobotMap;
import org.usfirst.frc199.OffseasonBot2016.commands.*;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Drivetrain extends Subsystem implements DashboardInterface {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController leftMotor = RobotMap.drivetrainLeftMotor;
    private final SpeedController rightMotor = RobotMap.drivetrainRightMotor;
    private final RobotDrive robotDrive = RobotMap.drivetrainRobotDrive;
    private final AnalogGyro gyro = RobotMap.drivetrainGyro;
    private final Encoder leftEncoder = RobotMap.drivetrainLeftEncoder;
    private final Encoder rightEncoder = RobotMap.drivetrainRightEncoder;
    private final DoubleSolenoid gearSolenoid = RobotMap.drivetrainDoubleSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    

    public PID drivePID = new PID("drivePID");
    public PID turnPID = new PID("turnPID");

    public double prevTurn = 0;
    double driveSpeed, driveAngle = 0;
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new TeleopDrive());
    	
    	
    }
    public void resetEncoder() {
    	leftEncoder.reset();
    	rightEncoder.reset();
    }
    public void resetGyro() {
    	gyro.reset();
    }
    public double getAngle() {
    	return gyro.getAngle();
    }
    public double getDistance() {
    	return (leftEncoder.get() + rightEncoder.get()) / 2;
    }
    public void driveAt(double drive, double turn) {
    	prevTurn = turn;
    	robotDrive.arcadeDrive(drive, turn);
    }
    public double getSpeed() {
    	return (leftEncoder.getRate() + rightEncoder.getRate()) / 2;
    }
    public double avgMotorSetSpeed() {
    	return (leftMotor.get() + rightMotor.get()) /2;
    }
    
    
	@Override
	public void displayData() {
		//Display any info that is useful
		//Sensor values
		
		putNumber("Left encoder distance", leftEncoder.get());
		putNumber("Right encoder distance", rightEncoder.get());
		putNumber("Left encoder speed", leftEncoder.getRate());
		putNumber("Right encoder speed", rightEncoder.getRate());
		putNumber("Average speed", getSpeed());
		putNumber("Average distance", getDistance());
		putNumber("Gyro", gyro.getAngle());
		putNumber("Left motor value", leftMotor.get());
		putNumber("Right motor value", rightMotor.get());
		
	}
	public void gradualDrive(double targetSpeed, double targetTurn) {
		driveSpeed = 0;
		driveAngle = 0;
		if(avgMotorSetSpeed() < targetSpeed) {
			driveSpeed = avgMotorSetSpeed() + 0.05;
		} else if(avgMotorSetSpeed() > targetSpeed){
			driveSpeed = avgMotorSetSpeed() - 0.05;
		} else if(avgMotorSetSpeed() == targetSpeed) {
			driveSpeed = avgMotorSetSpeed();
		}
		if(prevTurn < targetTurn) {
			driveAngle = prevTurn + 0.05;
		} else if(prevTurn > targetTurn) {
			driveAngle = prevTurn - 0.05;
		} else if(prevTurn == targetTurn) {
			driveAngle = prevTurn;
		}
//		prevTurn = driveAngle;
		driveAt(driveSpeed, driveAngle);		
	}
	public void shiftLow() {
		gearSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	public void shiftHigh() {
		gearSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	public void gearSolenoidSetZero() {
		gearSolenoid.set(DoubleSolenoid.Value.kOff);
	}

}


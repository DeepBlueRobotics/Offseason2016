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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Shooter extends Subsystem implements DashboardInterface {

    private final SpeedController shooterMotor = RobotMap.shooterShooterMotor;
    private final SpeedController loaderMotor = RobotMap.shooterLoaderMotor;
    private final Encoder shooterEncoder = RobotMap.shooterShooterEncoder;
    private final DoubleSolenoid hoodTiltPiston = RobotMap.shooterHoodTiltPiston;


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

	@Override
	public void displayData() {
		
	}
	
	public PID speed = new PID("ShooterSpeed");
	
	public boolean findShooterSpeedPID(){
		//?? I know this (setTarget) will repeat every time
		//the method is called, which is unnecessary,
		//but where else would I do it? Do I need to?
		//(I would assume so...)
		speed.update(shooterEncoder.get());
		if(speed.reachedTarget())
			return true;
		else return false;
	}
	
	
	
	//load shooter (motor full)
	//shooter motor (varying speed)
	//hoodTiltPiston: toggle, keep track of position
	
	/**
	 * loads the shooter
	 * */
	public void loadShooter(){
		loaderMotor.set(1);
	}
	
	public void setLoaderMotorToZero() {
		loaderMotor.set(0);
	}
	/**
	 * shoots the ball
	 * */
	public void shoot(double speed){
		shooterMotor.set(speed);
	}
	
	/**
	 * sets the hood position
	 * @param piston if true, piston is set up; if false, piston is set down
	 * */
	public void tiltHood(boolean piston){
		if(piston)
			hoodTiltPiston.set(DoubleSolenoid.Value.kForward);
		else hoodTiltPiston.set(DoubleSolenoid.Value.kReverse);

	}
	public void setSolenoidZero() {
		hoodTiltPiston.set(DoubleSolenoid.Value.kOff);
	}
	public void setShooterMotorZero() {
		shooterMotor.set(0);
	}
}


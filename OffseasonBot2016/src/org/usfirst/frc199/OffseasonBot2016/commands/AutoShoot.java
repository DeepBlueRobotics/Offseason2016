// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc199.OffseasonBot2016.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc199.OffseasonBot2016.Robot;

/**
 *
 */
public class AutoShoot extends Command {


	private Timer timer = new Timer();
	private boolean firstTime = true;
	
	
    public AutoShoot() {

        requires(Robot.shooter);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.start();
    	Robot.shooter.speed.setTarget(1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.shoot(Robot.shooter.speed.getOutput());
    	if(Robot.shooter.findShooterSpeedPID()){
    		if(firstTime){
    			timer.reset();
    			firstTime = false;
    		}
    		Robot.shooter.loadShooter();
    	}
    }
    
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return timer.get() > 1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setShooterMotorZero();
    	Robot.shooter.setLoaderMotorToZero();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

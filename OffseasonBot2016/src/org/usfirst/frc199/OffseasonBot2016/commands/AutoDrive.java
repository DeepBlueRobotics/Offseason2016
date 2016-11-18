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

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc199.OffseasonBot2016.Robot;
import org.usfirst.frc199.OffseasonBot2016.PID;

/**
 *
 */
public class AutoDrive extends Command {


	double target;
    public AutoDrive(double target) {

        requires(Robot.drivetrain);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.resetEncoder();
    	Robot.drivetrain.drivePID.setTarget(target);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.drivePID.update(Robot.drivetrain.getDistance());
    	Robot.drivetrain.driveAt(Robot.drivetrain.drivePID.getOutput(), 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.drivePID.reachedTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.driveAt(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

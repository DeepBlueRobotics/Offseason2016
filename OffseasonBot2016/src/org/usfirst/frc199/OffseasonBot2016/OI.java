// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc199.OffseasonBot2016;

import org.usfirst.frc199.OffseasonBot2016.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc199.OffseasonBot2016.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton intakeButton;
    public Joystick leftJoy;
    public JoystickButton shootButton;
    public JoystickButton mainTiltButton;
    public Joystick rightJoy;
    public JoystickButton runShooterButton;
    public JoystickButton loadShooterButton;
    public JoystickButton shootLowButton;
    public JoystickButton tiltHighButton;
    public JoystickButton tiltLowButton;
    public JoystickButton pivotIntakeButton;
    public JoystickButton intakeInButton;
    public JoystickButton intakeOutButton;
    public JoystickButton visionAlignButton;
    public Joystick manipulator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        manipulator = new Joystick(2);
        
        visionAlignButton = new JoystickButton(manipulator, 1);
        visionAlignButton.whenPressed(new VisionAlign());
        intakeOutButton = new JoystickButton(manipulator, 1);
        intakeOutButton.whileHeld(new RunIntakeOut());
        intakeInButton = new JoystickButton(manipulator, 1);
        intakeInButton.whileHeld(new RunIntakeIn());
        pivotIntakeButton = new JoystickButton(manipulator, 1);
        pivotIntakeButton.whenPressed(new PivotIntake());
        tiltLowButton = new JoystickButton(manipulator, 1);
        tiltLowButton.whenPressed(new ShooterTiltDown());
        tiltHighButton = new JoystickButton(manipulator, 1);
        tiltHighButton.whenPressed(new ShooterTiltUp());
        shootLowButton = new JoystickButton(manipulator, 1);
        shootLowButton.whileHeld(new ShootLow());
        loadShooterButton = new JoystickButton(manipulator, 1);
        loadShooterButton.whileHeld(new LoadShooter());
        runShooterButton = new JoystickButton(manipulator, 1);
        runShooterButton.whileHeld(new RunShooter());
        rightJoy = new Joystick(1);
        
        mainTiltButton = new JoystickButton(rightJoy, 1);
        mainTiltButton.whileHeld(new ToggleShootingTilt());
        shootButton = new JoystickButton(rightJoy, 1);
        shootButton.whenPressed(new AutoShoot());
        leftJoy = new Joystick(0);
        
        intakeButton = new JoystickButton(leftJoy, 1);
        intakeButton.whenPressed(new AutoIntake());


        // SmartDashboard Buttons
        SmartDashboard.putData("MainAutoMode", new MainAutoMode());
        SmartDashboard.putData("AutoDrive", new AutoDrive());
        SmartDashboard.putData("AutoTurn", new AutoTurn());
        SmartDashboard.putData("AutoDelay", new AutoDelay());
        SmartDashboard.putData("TeleopDrive", new TeleopDrive());
        SmartDashboard.putData("VisionAlign", new VisionAlign());
        SmartDashboard.putData("AutoShoot", new AutoShoot());
        SmartDashboard.putData("RunShooter", new RunShooter());
        SmartDashboard.putData("LoadShooter", new LoadShooter());
        SmartDashboard.putData("ShootLow", new ShootLow());
        SmartDashboard.putData("ToggleShootingTilt", new ToggleShootingTilt());
        SmartDashboard.putData("ShooterTiltUp", new ShooterTiltUp());
        SmartDashboard.putData("ShooterTiltDown", new ShooterTiltDown());
        SmartDashboard.putData("AutoIntake", new AutoIntake());
        SmartDashboard.putData("PivotIntake", new PivotIntake());
        SmartDashboard.putData("RunIntakeIn", new RunIntakeIn());
        SmartDashboard.putData("RunIntakeOut", new RunIntakeOut());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getLeftJoy() {
        return leftJoy;
    }

    public Joystick getRightJoy() {
        return rightJoy;
    }

    public Joystick getManipulator() {
        return manipulator;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}


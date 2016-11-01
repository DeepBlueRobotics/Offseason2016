
package org.usfirst.frc.team199.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    
    double target, kP, kI, kD, sumOfDistances, targetAngle;
    
    public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    
    RobotDrive robotDrive;
    SpeedController leftSpeedControl, rightSpeedControl;
    
    AnalogGyro analogGyro;
    
    int turnDirection;
    
    Joystick leftJoystick;
    Joystick rightJoystick;
    
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("target angle", 0);
        
        SmartDashboard.putNumber("target distance", 0);
        target = SmartDashboard.getNumber("target distance");
        
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);
        leftEncoder.setDistancePerPulse(0.0002);
        rightEncoder.setDistancePerPulse(0.0002);
        
        leftSpeedControl = new Victor(0);
        rightSpeedControl = new Victor(1);
        
        robotDrive = new RobotDrive(leftSpeedControl, rightSpeedControl);
        
        analogGyro = new AnalogGyro(0);
        
        leftJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		sumOfDistances = 0;
		leftEncoder.reset();
		rightEncoder.reset();
		
        kP = SmartDashboard.getNumber("kP"); 
        kI = SmartDashboard.getNumber("kI");
        kD = SmartDashboard.getNumber("kD");
        
        targetAngle = SmartDashboard.getNumber("target angle");
        
        analogGyro.reset();
        
        turnDirection = Integer.signum((int) targetAngle);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}
    	sumOfDistances += getDistance();
    	robotDrive.arcadeDrive((target - getDistance()) * kP + sumOfDistances * kI - getRate() * kD, turnDirection());
    }
    
    public int turnDirection () {
    	if (Math.abs(analogGyro.getAngle()) < Math.abs(targetAngle)) {
    		return turnDirection;
    	} else {
    		return 0;
    	}
    }
    
    private double getDistance() {
    	return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
    }
    
    private double getRate() {
    	return (leftEncoder.getRate() + rightEncoder.getRate()) / 2;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	robotDrive.arcadeDrive(leftJoystick.getY(), rightJoystick.getX());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

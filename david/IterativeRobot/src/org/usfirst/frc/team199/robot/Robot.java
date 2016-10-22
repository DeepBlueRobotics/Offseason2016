
package org.usfirst.frc.team199.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
    private double target, atarget = 0;
    private double kI,kD,kP = 0;
    private double akI,akD,akP = 0;
	private double integral, aintegral = 0;
	private Encoder encoderLeft;
	private Encoder encoderRight;
	private AnalogGyro gyro;
	private Victor leftMotor1, leftMotor2, leftMotor3;
	private Victor rightMotor1, rightMotor2, rightMotor3;
	private RobotDrive drive;
	private Joystick joystickl, joystickr;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
		encoderLeft = new Encoder(0,1);
		encoderRight = new Encoder(2,3);
		
		leftMotor1 = new Victor(0);
		leftMotor2 = new Victor(1);
		leftMotor3 = new Victor(2);
		rightMotor1 = new Victor(3);
		rightMotor2 = new Victor(4);
		rightMotor3 = new Victor(5);
		drive = new RobotDrive(leftMotor1, rightMotor1);
		gyro = new AnalogGyro(0);
		
		joystickl = new Joystick(0);
		joystickr = new Joystick(1);
		
		SmartDashboard.putNumber("kI", kI);
    	SmartDashboard.putNumber("kD", kD);
    	SmartDashboard.putNumber("kP", kP);
    	SmartDashboard.putNumber("akI", akI);
    	SmartDashboard.putNumber("akD", akD);
    	SmartDashboard.putNumber("akP", akP);
    	SmartDashboard.putNumber("target", target);
    	SmartDashboard.putNumber("atarget", atarget);;
        SmartDashboard.putString("drive_type", "drive");
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
		
		//reset encoders and gyro
		gyro.reset();
		encoderRight.reset();
		encoderLeft.reset();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//switch(autoSelected) {
    	//case customAuto:
    		//Put custom auto code here   
        //    break;
    	//case defaultAuto:
    	//default:
    		//Put default auto code here
       //     break;
    	//}
    	resetSmartDashboard();
    	if (SmartDashboard.getString("drive_type").equalsIgnoreCase("drive")) {
    		drive.tankDrive(PIDDrive(target), PIDDrive(target));
    	} else {
    		drive.tankDrive(PIDTurn(atarget), -PIDTurn(atarget));
    	}
    	SmartDashboard.putNumber("encoderLeft", encoderLeft.get());
    	SmartDashboard.putNumber("encoderRight", encoderRight.get());
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //drive.tankDrive(joystickl.getY(), joystickr.getY());
        leftMotor1.set(SmartDashboard.getNumber("speed"));
        leftMotor2.set(SmartDashboard.getNumber("speed"));
        leftMotor3.set(SmartDashboard.getNumber("speed"));
        rightMotor1.set(SmartDashboard.getNumber("speed"));
        rightMotor2.set(SmartDashboard.getNumber("speed"));
        rightMotor3.set(SmartDashboard.getNumber("speed"));

    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    private double PIDDrive(double target) {
    	double error = target - avg(encoderRight.get(), encoderLeft.get());
    	double speed = avg(leftMotor1.getSpeed(),rightMotor1.getSpeed());
    	integral += error;
    	return kP*error - kD*speed + kI*integral;
    }
    private double PIDTurn(double atarget) {
    	double aerror = atarget - gyro.getAngle();
    	double aspeed = gyro.getRate();
    	aintegral += aerror;
    	return akP*aerror - akD*aspeed + akI*integral;
    }
    
    private double avg(double a, double b) {
    	return (a+b)/2;
    }
    private void resetSmartDashboard() {
    	kI = SmartDashboard.getNumber("kI");
    	kD = SmartDashboard.getNumber("kD");
    	kP = SmartDashboard.getNumber("kP");
    	akI = SmartDashboard.getNumber("akI");
    	akD = SmartDashboard.getNumber("akD");
    	akP = SmartDashboard.getNumber("akD");
    	target = SmartDashboard.getNumber("target");
    	atarget = SmartDashboard.getNumber("atarget");
    	encoderLeft.setDistancePerPulse(SmartDashboard.getNumber("distance/pulse"));
    	encoderRight.setDistancePerPulse(SmartDashboard.getNumber("distance/pulse"));
    }
    
}

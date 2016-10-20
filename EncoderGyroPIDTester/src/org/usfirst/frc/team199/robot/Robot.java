
package org.usfirst.frc.team199.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
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
    Gyro gyro;
    Encoder encoder1;
    Encoder encoder2;
    RobotDrive roboDrive;
	int stepCounter;
	double integral;
	double angularIntegral;
	private double kP;
	private double kI;
	private double kD;
	private double akP;
	private double akI;
	private double akD;
    private VictorSP motorL;
    private VictorSP motorR;
    private Joystick stickL;
    private Joystick stickR;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        gyro = new AnalogGyro(0);
        gyro.reset();
        encoder1 = new Encoder(0, 1);
        encoder2 = new Encoder(2, 3);
        roboDrive = new RobotDrive(0, 1);
        motorL = new VictorSP(0);
        motorR = new VictorSP(1);
        stickL = new Joystick(0);
        stickR = new Joystick(1);
        SmartDashboard.putNumber("Step Number", 0);
        SmartDashboard.putNumber("Error Graph", 0);
        
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
        integral = 0;
		angularIntegral = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	setConstants();
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("encoder", avgDistance());
		
		//Turning
    	if (SmartDashboard.getNumber("Step Number") == 1) roboDrive.arcadeDrive(0, PIDTurn(-180));
    	if (SmartDashboard.getNumber("Step Number") == 2) roboDrive.arcadeDrive(0, PIDTurn(-90));
    	if (SmartDashboard.getNumber("Step Number") == 3) roboDrive.arcadeDrive(0, PIDTurn(0));
    	if (SmartDashboard.getNumber("Step Number") == 4) roboDrive.arcadeDrive(0, PIDTurn(90));
    	if (SmartDashboard.getNumber("Step Number") == 5) roboDrive.arcadeDrive(0, PIDTurn(180));
    	//Driving
    	if (SmartDashboard.getNumber("Step Number") == 6) roboDrive.arcadeDrive(PIDDrive(-4), 0);
    	if (SmartDashboard.getNumber("Step Number") == 7) roboDrive.arcadeDrive(PIDDrive(-2), 0);    	
    	if (SmartDashboard.getNumber("Step Number") == 8) roboDrive.arcadeDrive(PIDDrive(0), 0);
    	if (SmartDashboard.getNumber("Step Number") == 9) roboDrive.arcadeDrive(PIDDrive(2), 0);
    	if (SmartDashboard.getNumber("Step Number") == 10) roboDrive.arcadeDrive(PIDDrive(4), 0);
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	roboDrive.tankDrive(stickL.getThrottle(), stickR.getThrottle());
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
    	SmartDashboard.putNumber("encoder", avgDistance());

    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    }
    
    public void setConstants() {
    	if (SmartDashboard.getBoolean("changeReq")) {
    		encoder1.setDistancePerPulse(SmartDashboard.getNumber("Distance Per Pulse1"));
    		encoder2.setDistancePerPulse(SmartDashboard.getNumber("Distance Per Pulse2"));
    		kP = SmartDashboard.getNumber("kP");
    		kI = SmartDashboard.getNumber("kI");
    		kD = SmartDashboard.getNumber("kD");
    		akP = SmartDashboard.getNumber("akP");
    		akI = SmartDashboard.getNumber("akI");
    		akD = SmartDashboard.getNumber("akD");
    	}
    }
    
    public double avgDistance() {
    	return (encoder1.getDistance() + encoder2.getDistance()) / 2;
    }
    
    public double avgSpeed() {
    	return (encoder1.getRate() + encoder2.getRate()) / 2;
    }
    
    public double PIDTurn(double targetAngle) {
    	double errorAngle = targetAngle - gyro.getAngle();
    	double angularSpeed = gyro.getRate();
    	angularIntegral += errorAngle;
    	SmartDashboard.putNumber("Error Angle Graph", errorAngle);
    	return errorAngle * akP - angularSpeed * akD + angularIntegral * akI;
    }
    
    public double PIDDrive(double target) {
    	double error = target - avgDistance();
    	double speed = avgSpeed();
    	integral += error;
    	SmartDashboard.putNumber("Error Graph", error);
    	return error * kP - speed * kD + integral * kI;
    	
    }
}

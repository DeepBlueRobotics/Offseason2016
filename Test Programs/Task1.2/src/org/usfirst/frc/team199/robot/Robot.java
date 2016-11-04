
package org.usfirst.frc.team199.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
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
    PowerDistributionPanel panel = new PowerDistributionPanel();	
    
    private static Encoder rightEncoder = new Encoder(1, 0);
    private static Encoder leftEncoder = new Encoder(2, 3);
    
//    private static Talon lMotor1 = new Talon(3);
//    private static Talon lMotor2 = new Talon(4);
//    private static Talon lMotor3 = new Talon(5);
    
    private static Talon lMotor1 = new Talon(0);
    private static Talon rMotor1 = new Talon(1);
    private RobotDrive drive = new RobotDrive(lMotor1, rMotor1);
//    private static Talon rMotor3 = new Talon(2);
    
    //private static RobotDrive robotDrive = new RobotDrive(leftMotor, rightMotor);
	Timer tim = new Timer();

    
    private static Joystick rightJoy = new Joystick(1);
    private static Joystick leftJoy = new Joystick(2);
    private static AnalogGyro gyro = new AnalogGyro(0);
    
    private double totalDistance;
    private double totalDegrees;
    private double kpDistance;
    private double kpDegrees;
    private double kdDistance;
    private double kdDegrees;
    private double kiDistance;
    private double kiDegrees;
    private double angle;
    private boolean resetTim = false;
    private double rate;
    
    /**
     * Updates the value of the encoder in SmartDashboard
     * to the current average of the left and right encoder values
     * */
    public void update(){
    	SmartDashboard.putNumber("encoder", getEncoder());
    	SmartDashboard.putNumber("right encoder", rightEncoder.get());
    	SmartDashboard.putNumber("left encoder", leftEncoder.get());
    	SmartDashboard.putNumber("Left Encoder Speed", rightEncoder.getRate());
    	SmartDashboard.putNumber("Right Encoder Speed", leftEncoder.getRate());
    	SmartDashboard.putNumber("Port0 Current", panel.getCurrent(0));
    	SmartDashboard.putNumber("Port1 Current", panel.getCurrent(1));
    	SmartDashboard.putNumber("Port2 Current", panel.getCurrent(2));
    	SmartDashboard.putNumber("Port3 Current", panel.getCurrent(3));
    	SmartDashboard.putNumber("Port4 Current", panel.getCurrent(4));
    	SmartDashboard.putNumber("Port5 Current", panel.getCurrent(5));
    	SmartDashboard.putNumber("Port6 Current", panel.getCurrent(6));
    	SmartDashboard.putNumber("Port7 Current", panel.getCurrent(7));
    	SmartDashboard.putNumber("Port8 Current", panel.getCurrent(8));
    	SmartDashboard.putNumber("Port9 Current", panel.getCurrent(9));
    	SmartDashboard.putNumber("Port10 Current", panel.getCurrent(10));
    	SmartDashboard.putNumber("Port11 Current", panel.getCurrent(11));
    	SmartDashboard.putNumber("Port12 Current", panel.getCurrent(12));
    	SmartDashboard.putNumber("Port13 Current", panel.getCurrent(13));
    	SmartDashboard.putNumber("Port14 Current", panel.getCurrent(14));
    	SmartDashboard.putNumber("Port15 Current", panel.getCurrent(15));
    	
    }
    
    /**
     * @return the ratio of distance traveled to the encoder's value
     * */
    public double setLeftEncoderRatio(double distanceTraveled){
    	return  distanceTraveled/SmartDashboard.getNumber("left encoder");
    }
    public double setRightEncoderRatio(double distanceTraveled){
    	return  distanceTraveled/SmartDashboard.getNumber("right encoder");
    }
    
    /**
     * PID driving forward/backward
     * */
    public void autoDrive(double speed){
    	drive.tankDrive(speed, speed);
    }
    
    /**
     * PID turning
     * */
    public void autoTurn(double angle){
    	int sign = (int) (angle/Math.abs(angle));
    	drive.tankDrive(sign, -sign);
    }
    
    /**
     * drives all 3 right motors as one
     * @param speed speed of right motors
     * */
//    public void driveRight(double speed){
//    	rMotor1.set(speed);
//    	rMotor2.set(speed);
//    	rMotor3.set(speed);
//    }
    
    /**
     * drives all 3 left motors as one
     * @param speed speed of left motors
     * */
//    public void driveLeft(double speed){
//    	lMotor1.set(speed);
//    	lMotor2.set(speed);
//    	lMotor3.set(speed);
//    }
    /**
     * Drives an individual motor
     * @param port port of motor driven*/
//    public void driveIndividualMotor(int port){
//    	switch (port) {
//    	case 0: lMotor1.set(.2); break;
//    	case 1: lMotor2.set(.2); break;
//    	case 2: lMotor3.set(.2); break;
//    	case 3: rMotor1.set(.2); break;
//    	case 4: rMotor2.set(.2); break;
//    	case 5: rMotor3.set(.2); break;
//    	default: driveRight(0); driveLeft(0); break;
//    	}
//    }
    
    /**
     * drive 6-motor robot using joysticks
     * */
    public void driveTeleop(){
//    	driveLeft(leftJoy.getThrottle());
//    	driveRight(rightJoy.getThrottle());
    	drive.tankDrive(leftJoy.getY(), rightJoy.getY());
    }
    
    /**
     * @return the value of the gyroscope
     * */
    public double getGyro(){
    	return gyro.getAngle();
    }
    
    /**
     * @return the average of the left and right encoder values
     * */
    public double getEncoder(){
    	return (leftEncoder.get() + rightEncoder.get())/2;
    }
    
    /**
     * Resets variables totalDistance and totalDegrees to zero
     * */
    public void resetTotals() {
    	totalDistance = 0;
    	totalDegrees = 0;
    }
    
    /**
     * use for PID
     * @param target target distance you want the robot to travel
     * @return necessary motor speed
     * */
    public double getMotorOutputEncoder(double target){
    	double error = target - getEncoder();
    	double traveled = getEncoder();
    	double speed = (leftEncoder.getRate() + rightEncoder.getRate())/2;
    	totalDistance += traveled;
    	return error*kpDistance - speed*kdDistance + totalDistance*kiDistance;
    }
    
    /**
     * use for PID
     * @param target target angle you want the robot to turn (in degrees)
     * @return necessary motor speed
     * */
    public double getMotorOutputGyro(double target){
    	double error = target - getGyro();
    	double traveled = getGyro();
    	double speed = gyro.getRate();
    	totalDegrees += traveled;
    	return error*kpDegrees - speed*kdDegrees + totalDegrees*kiDegrees;
    }
    
    /**
     * Sets the constants used for PID control to their values in SmartDashboard
     * */
    public void setConstants(){
    	kpDistance = SmartDashboard.getNumber("kp Distance");
        kpDegrees = SmartDashboard.getNumber("kp Degrees");
        kdDistance = SmartDashboard.getNumber("kd Distance");
        kdDegrees = SmartDashboard.getNumber("kd Degrees");
        kiDistance = SmartDashboard.getNumber("ki Distance");
        kiDegrees = SmartDashboard.getNumber("ki Degrees");
    }
    
    public void gyroDrift() {
    	if(tim.get() < 3) {
    		angle = gyro.getAngle();
    	}
    	rate = gyro.getAngle() / tim.get();
    	SmartDashboard.putNumber("rate of gyro drift", rate);
    }
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        SmartDashboard.putNumber("kp Distance", 0);
        SmartDashboard.putNumber("kp Degrees", 0);
        SmartDashboard.putNumber("kd Distance", 0);
        SmartDashboard.putNumber("kd Degrees", 0);
        SmartDashboard.putNumber("ki Distance", 0);
        SmartDashboard.putNumber("ki Degrees", 0);
        
        SmartDashboard.putNumber("target angle", 0);
        SmartDashboard.putNumber("target distance", 0);
        
        SmartDashboard.putNumber("motor port", 0);
        SmartDashboard.putBoolean("joyRide", false);
        
        tim.reset();
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
		setConstants();
		resetTotals();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here
            autoTurn(getMotorOutputGyro(SmartDashboard.getNumber("target angle")));
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
    		autoDrive(getMotorOutputEncoder(SmartDashboard.getNumber("target distance")));
            break;
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
//    	driveTeleop();
//    	driveIndividualMotor((int)SmartDashboard.getNumber("motor port"));
    	update();
    	SmartDashboard.putNumber("left encoder ratio", setLeftEncoderRatio(24));
    	SmartDashboard.putNumber("right encoder ratio", setRightEncoderRatio(24));
    	SmartDashboard.putNumber("gyro", getGyro());
    	SmartDashboard.putNumber("Left Joy", leftJoy.getY());
    	SmartDashboard.putNumber("Right Joy", rightJoy.getY());
    	
//    	driveLeft(SmartDashboard.getNumber("motor port"));
//    	driveRight(SmartDashboard.getNumber("motor port"));
    	if(SmartDashboard.getBoolean("joyRide")) {
    		drive.arcadeDrive(leftJoy.getY(), rightJoy.getX());
    	} else {
    		drive.tankDrive(-SmartDashboard.getNumber("motor port"), -SmartDashboard.getNumber("motor port"));
    	}
    	
    	
    	
//        if(SmartDashboard.getBoolean("test gyro drift")) {
//        	if(!resetTim) {
//        		tim.reset();
//        		resetTim = !resetTim;
//        	}
//        	gyroDrift();
//        	
//        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

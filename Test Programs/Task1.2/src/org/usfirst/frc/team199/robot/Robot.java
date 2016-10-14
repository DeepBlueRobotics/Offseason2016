
package org.usfirst.frc.team199.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
	
    
    private static Encoder rightEncoder = new Encoder(0, 1);
    private static Encoder leftEncoder = new Encoder(2, 3);
    
    private static Talon lMotor1 = new Talon(0);
    private static Talon lMotor2 = new Talon(1);
    private static Talon lMotor3 = new Talon(2);
    
    private static Talon rMotor1 = new Talon(3);
    private static Talon rMotor2 = new Talon(4);
    private static Talon rMotor3 = new Talon(5);
    
    //private static RobotDrive robotDrive = new RobotDrive(leftMotor, rightMotor);
    
    private static Joystick rightJoy = new Joystick(1);
    private static Joystick leftJoy = new Joystick(2);
    private static AnalogGyro gyro = new AnalogGyro(6);
    
    private double totalDistance;
    private double totalDegrees;
    private double kpDistance;
    private double kpDegrees;
    private double kdDistance;
    private double kdDegrees;
    private double kiDistance;
    private double kiDegrees;
    
    /**
     * Updates the value of the encoder in SmartDashboard
     * to the current average of the left and right encoder values
     * */
    public void update(){
    	SmartDashboard.putNumber("encoder", getEncoder());
    }
    
    /**
     * @return the ratio of distance traveled to the encoder's value
     * */
    public double setEncoderRatio(double distanceTraveled){
    	return  distanceTraveled/SmartDashboard.getNumber("encoder");
    }
    
    /**
     * PID driving forward/backward
     * */
    public void autoDrive(double speed){
    	driveRight(speed);
    	driveLeft(speed);
    }
    
    /**
     * PID turning
     * */
    public void autoTurn(double angle){
    	int sign = (int) (angle/Math.abs(angle));
    	driveLeft(sign);
    	driveRight(-sign);
    }
    
    /**
     * drives all 3 right motors as one
     * @param speed speed of right motors
     * */
    public void driveRight(double speed){
    	rMotor1.set(speed);
    	rMotor2.set(speed);
    	rMotor3.set(speed);
    }
    
    /**
     * drives all 3 left motors as one
     * @param speed speed of left motors
     * */
    public void driveLeft(double speed){
    	lMotor1.set(speed);
    	lMotor2.set(speed);
    	lMotor3.set(speed);
    }
    
    /**
     * drive 6-motor robot using joysticks
     * */
    public void driveTeleop(){
    	driveLeft(leftJoy.getThrottle());
    	driveRight(rightJoy.getThrottle());
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
    	driveTeleop();
    	update();
    	SmartDashboard.putNumber("encoder ratio", setEncoderRatio(12));
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}

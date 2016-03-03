
/****************************************
 * 
 *	Iterative Based JAVA for 2016 FRC Season
 *	@author CyberCoyotes
 *
 ****************************************/

package org.usfirst.frc.team3603.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	/* Assigns speed controllers to names and PWN numbers */

	Joystick xbox = new Joystick(0); // XBOX controller for driving - USB 0

	Victor left = new Victor(0); // PWM channel 0 for left drive train

	Victor right = new Victor(1); // PWN channel 1 for right motor drive train

	RobotDrive maindrive = new RobotDrive(left, right);

	Joystick xbox2 = new Joystick(5); // XBOX controller for manipulating - USB
										// 5

	Talon lift = new Talon(2); // scissor lift screw drive motor

	Talon shooter = new Talon(3); // shooter wheels

	Victor winch = new Victor(4); // CIM winch 1 motor

	Victor winch2 = new Victor(5); // CIM winch 2 motor

	Compressor single = new Compressor();

	DoubleSolenoid doublesol = new DoubleSolenoid(1, 2); // Shooter pneumatics
															// lifter

	DoubleSolenoid doublesol2 = new DoubleSolenoid(6, 7); // Shooter pneumatic
															// pusher
	DoubleSolenoid doublesol3 = new DoubleSolenoid(4, 5);
	 AnalogInput ultrasonic = new AnalogInput(0);
	 AnalogGyro gyro = new AnalogGyro(1);

	// CameraServer server;
	int session;
	Image frame;
	Timer Timer = new Timer();

	public void robotInit() {

		single.start();
		Timer.start();
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

	}


	public void autonomousInit() {
		Timer.reset();

		Timer.start();

	}

	public void autonomousPeriodic() {
		while (isAutonomous()) {
			maindrive.setSafetyEnabled(false);

			double gyroangle = gyro.getAngle();
			double distanceinches = ultrasonic.getValue() / 9.766;
			SmartDashboard.putNumber("Timer during auton", Timer.get());

			SmartDashboard.putNumber("Gyro in auton", gyroangle);
			SmartDashboard.putString("Shooter ON/OFF", "OFF");
			while (Timer.get() < 6.0) {
				maindrive.tankDrive(.5, .5);
				SmartDashboard.putNumber("Timer during auton", Timer.get());

			}
			while (Timer.get() < 10.0) {
				maindrive.tankDrive(.75, .75);
				SmartDashboard.putNumber("Timer during auton", Timer.get());

			}
			while (distanceinches > 50) {
				maindrive.tankDrive(.5, .5);
				distanceinches = ultrasonic.getValue() / 9.766;
				SmartDashboard.putNumber("distanceinches in auton", distanceinches);

			}
			SmartDashboard.putNumber("gyro before loop", gyro.getAngle());
			while (gyroangle < 40) {
				gyroangle = gyro.getAngle();
				maindrive.tankDrive(.5, -.5);
				SmartDashboard.putNumber("Gyro in auton", gyroangle);

			}
			while (Timer.get() < 8.0) {
				doublesol.set(DoubleSolenoid.Value.kForward);
				doublesol2.set(DoubleSolenoid.Value.kReverse);
			}
			while (Timer.get() < 10.0) {
				shooter.set(1.0);
			}

		}
		
	}

	public void teleopPeriodic() {
		while (isOperatorControl() && isEnabled()) {
			double distanceinches = ultrasonic.getValue() / 9.766;
			SmartDashboard.putNumber("distance in inches", distanceinches);

			// NIVision.IMAQdxStartAcquisition(session);
			NIVision.IMAQdxGrab(session, frame, 1);
			CameraServer.getInstance().setImage(frame);
			// NIVision.IMAQdxStopAcquisition(session);
			/* XBOX 1 DRIVER CODE */
			SmartDashboard.putNumber("Timer value", Timer.get());
			double xboxmagnitude1 = xbox.getRawAxis(1);
			double xboxmagnitude5 = xbox.getRawAxis(5);

			double all = xboxmagnitude1 + xboxmagnitude5;
			
			
			
			/****************************************
			* BLACK XBOX CONTROLLER
			* DRIVER + SHOOT BALL
			***************************************/

			if (all > .2 || all < -.2) { // || 
				maindrive.tankDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(5));
			}
			SmartDashboard.putNumber("Timer value", Timer.get());


			if (xbox.getRawButton(1)) { 		// A BUTTON
				maindrive.tankDrive(-.5, .5);	// Spin left slow
			}

			if (xbox.getRawButton(2)) { 		// B BUTTON
				maindrive.tankDrive(.5, -.5);	// Spin right slow
			}
			if (xbox.getRawButton(3)) { 		// X BUTTON
				maindrive.tankDrive(.5, .5);	// Drive Forward Slow
			}

			if (xbox.getRawButton(4)) { 		// Y BUTTON
				maindrive.tankDrive(-.5, -.5);	// Drive Backward Slow
			}

			/*** 
			 * MOVED BACK TO SECONDARY USB (Axis 2 and Axis 3) BUT NEEDS CONFIRMATION
			 */
//			if (xbox.getRawButton(5)) {			// LEFT BUMPER
//				shooter.set(.75);				// ball wheel spin in 
//			} else if (xbox.getRawButton(6)) {	// RIGHT Bumper
//				shooter.set(-1.0);				// ball wheel spin out - requires pneumatic to fire ball
//			} else {
//				shooter.stopMotor();
//			}
			
			/* Shooter Pneumatics Pusher */
			if (xbox.getRawButton(6)) { 						// RIGHT BUMPER
				doublesol2.set(DoubleSolenoid.Value.kReverse);	// Push Ball into shooter wheels
																// Pneumatic should retract on its own

			} else {
				doublesol2.set(DoubleSolenoid.Value.kForward);
			}


			if (xbox.getRawAxis(3) > 0.0) { 	// TRIGGER RIGHT
												// Drive right side forward with variable speed at
												// axis command
				maindrive.tankDrive(0, xbox.getRawAxis(3));		
				if (xbox.getRawAxis(2) > 0.0) {		
												
				maindrive.tankDrive(1.0, 1.0);
				}
			}

			if (xbox.getRawAxis(2) > 0.0) { // TRIGGER LEFT Drive left side
											// forward with variable speed at
											// axis command
				maindrive.tankDrive(xbox.getRawAxis(2), 0);
				if (xbox.getRawAxis(3) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}
			if (xbox.getRawButton(7)) { // BACK BUTTON 7 Drive backwards at full
										// speed
				maindrive.tankDrive(-1.0, -1.0);
			}
			if (xbox.getRawButton(8)) { // START BUTTON 8 Drive forward at full speed
				maindrive.tankDrive(1.0, 1.0);
			}

		
			/****************************************
			* AFTERGLOW XBOX CONTROLLER
			* MANIPULATOR
			***************************************/

			if (xbox2.getRawButton(1)) { 		// A BUTTON 
				lift.set(-.75);					// Scissor Lift Motor DOWN
				SmartDashboard.putNumber("lift value", lift.get());

			} else if (xbox2.getRawButton(2)) {	// B BUTTON Scissor Lift Motor
				lift.set(.5);					// Scissor Lift Motor UP
				SmartDashboard.putNumber("lift value", lift.get());

			} else {
				lift.stopMotor();
				SmartDashboard.putNumber("lift value", lift.get());

			}
			
			/* Arm DoubleSolenoid */
			if (xbox2.getRawButton(4)) { 						// X BUTTON 
				doublesol3.set(DoubleSolenoid.Value.kForward); 	// should raise arm - NEEDS VERIFICATION
				} else if (xbox2.getRawButton(3)) { 			// Y BUTTON
				doublesol3.set(DoubleSolenoid.Value.kReverse);	// should lower arm - NEEDS VERIFICATION
				}

			if (xbox2.getRawButton(8)) { 		// START BUTTON Winch
				winch.set(1.0);					// CIM 1 Pull Robot Up
				SmartDashboard.putNumber("winch value", winch.get());
				winch2.set(1.0);				// CIM 2 Pull Robot Up - ALWAYS KEEP SAME VALUE AS CIM 1 for Button
				SmartDashboard.putNumber("winch2 value", winch.get());

			} else if (xbox2.getRawButton(7)) { // BACK BUTTON Winch
				winch.set(-1.0);				// CIM 1 Let out line
				SmartDashboard.putNumber("winch value", winch.get());
				winch2.set(-1.0);				// CIM 2 Let out line - ALWAYS KEEP SAME VALUE as CIM 1 for Button
				SmartDashboard.putNumber("winch2 value", winch.get());

			} else {
				winch.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());
				winch2.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}

			/* Shooter Pneumatics Lifter */
			if (xbox2.getRawButton(6)) { // RIGHT BUMPER double solenoid Lift
											// Shooter
				doublesol.set(DoubleSolenoid.Value.kForward);

			} else if (xbox2.getRawButton(5)) { // LEFT BUMPER double solenoid
												// Lower Shooter
				doublesol.set(DoubleSolenoid.Value.kReverse);
			}

			
			// NEED TO CHANGE TO BOOLEAN
//			if (xbox2.getRawAxis(2)) {
//				shooter.set(.75);
				
//			} else if (xbox2.getRawAxis(3)) {
//				shooter.set(-1.0);
//			} else {
//				shooter.stopMotor();
//			}

			
			/********
			* CODE BELOW SHOULD BE SAFE TO DELETE
			*/
//			/* Shooter Pneumatics Pusher */ MOVED to USB 1 DRIVER BUTTON 6
//			if (xbox2.getRawButton(10)) { // THUMBSTICK BUTTON DOWN Ball Pusher
//											// Retractor
//				doublesol2.set(DoubleSolenoid.Value.kReverse);
//
//			} else { // THUMBSTICK BUTTON DOWN Ball
//						// Pusher
//				doublesol2.set(DoubleSolenoid.Value.kForward);
//			}

			

	

			
			edu.wpi.first.wpilibj.Timer.delay(0.005); // WHAT DOES THIS CODE DO? 

		}

	}

	public void testPeriodic() {

	}

}


/****************************************
 * 
 *	Iterative Based JAVA for 2016 FRC Season
 *	@author CyberCoyotes
 *
 ****************************************/

package org.usfirst.frc.team3603.robot;

import com.ni.vision.NIVision;

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

/* Added for WebCam */
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;
import edu.wpi.first.wpilibj.CameraServer;


public class Robot extends IterativeRobot {
/* Assigns speed controllers to names and PWN numbers */
	
	Joystick xbox = new Joystick(0);					// XBOX controller for driving - USB 0

	Victor left = new Victor(0);						// PWM channel 0 for left drive train

	Victor right = new Victor(1);						// PWN channel 1 for right motor drive train

	RobotDrive maindrive = new RobotDrive(left, right);

	Joystick xbox2 = new Joystick(1);					// XBOX controller for manipulating - USB 1

	Talon lift = new Talon(2);							// scissor lift screw drive motor

	Talon shooter = new Talon(3); 						// shooter wheels

	Victor winch = new Victor(4); 						// CIM winch 1 motor

	Victor winch2 = new Victor(5); 						// CIM winch 2 motor

	//	Victor arm = new Victor(5);						// Arm manipulator motor - REPLACED WITH CIM 2016-02-19

	Compressor single = new Compressor();

	DoubleSolenoid doublesol = new DoubleSolenoid(1, 2); // Shooter pneumatics lifter 

	DoubleSolenoid doublesol2 = new DoubleSolenoid(6, 7); // Shooter pneumatic pusher


	Timer Timer = new Timer();
	int session; // Added for Camera
    Image frame; // Added for Camera
    
	public void robotInit() {
		single.start();
		Timer.start();
		
		// USB Webcamera Init Code
	    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
	}


	/*	AUTONONMOUS Code */
	//	GOAL 1: Drive under low bar with a ball 
	//	GOAL 2: Turn and target goal
	//	GOAL 3: Shoot ball for a score
	//
	//	ASSUMPTIONS
	//	Driving under the low bar
	//	Shooting (High or Low?)
	public void autonomousPeriodic() {

	}

	/*
	 *	TELEOP Code
	 *
	*/

	public void teleopPeriodic() {
		while (isOperatorControl() && isEnabled()) {

			/* XBOX 1 DRIVER CODE */
			SmartDashboard.putNumber("Timer value", Timer.get());
			double xboxmagnitude1 = xbox.getRawAxis(1);
			double xboxmagnitude5 = xbox.getRawAxis(5);

			double all = xboxmagnitude1 + xboxmagnitude5;

			if (all > .2 || all < -.2) { 				// || what does this code do? 
				maindrive.tankDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(5));
			}
			SmartDashboard.putNumber("Timer value", Timer.get());



			if (xbox.getRawButton(3)) { 				// X BUTTON Drive Forward
				maindrive.tankDrive(.5, .5);
			}

			if (xbox.getRawButton(4)) { 				// Y BUTTON Drive backward
				maindrive.tankDrive(-.5, -.5);
			}

			if (xbox.getRawButton(1)) { 				// A BUTTON Spin slow left DO WE NEED THIS?
				maindrive.tankDrive(-.5, .5);
			}

			if (xbox.getRawButton(2)) { 				// B BUTTON Spin slow right DO WE NEED THIS?
				maindrive.tankDrive(.5, -.5);
			}

			if (xbox.getRawButton(6)) { 				// BUMPER RIGHT Drive right side forward, turn left
				maindrive.tankDrive(1.0, -1.0);
			}

			if (xbox.getRawButton(5)) {					// BUMPER LEFT Drive left side forward, turn right
				maindrive.tankDrive(-1.0, 1.0);
			}

			if (xbox.getRawAxis(3) > 0.0) {				// TRIGGER RIGHT Drive right side forward with variable speed at axis command
				maindrive.tankDrive(0, xbox.getRawAxis(3));
				if (xbox.getRawAxis(2) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}


			if (xbox.getRawAxis(2) > 0.0) {				// TRIGGER LEFT Drive left side forward with variable speed at axis command
				maindrive.tankDrive(xbox.getRawAxis(2), 0);
				if (xbox.getRawAxis(3) > 0.0) {			
					maindrive.tankDrive(1.0, 1.0);
				}
			}			

			/* XBOX 2 Manipulator Code*/
			
														// X BUTTON future code for tomahawk piston

														// Y BUTTON future code for tomahawk piston


			if (xbox2.getRawButton(1)) {				// A BUTTON Scissor Lift Motor UP
				lift.set(.5); 
				SmartDashboard.putNumber("lift value", lift.get());

			} else if (xbox2.getRawButton(2)) {		// B BUTTON Scissor Lift Motor DOWN
				lift.set(-.75); 
				SmartDashboard.putNumber("lift value", lift.get());

			} else {
				lift.stopMotor();
				SmartDashboard.putNumber("lift value", lift.get());

			}

			/* First winch CIM  */
			if (xbox2.getRawButton(8)) { 				// START BUTTON Winch 1 - pull robot up
				winch.set(1);
				SmartDashboard.putNumber("winch value", winch.get());

			} else if (xbox2.getRawButton(7)) { 		// BACK BUTTON Winch 1 - let out line
				winch.set(-.5);
				SmartDashboard.putNumber("winch value", winch.get());

			} else {
				winch.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}

			/* Second Winch CIM  */
			if (xbox2.getRawButton(8)) { 				// START BUTTON Winch 2 - pull robot up
				winch2.set(1);
				SmartDashboard.putNumber("winch2 value", winch.get());

			} else if (xbox2.getRawButton(7)) { 		// BACK BUTTON Winch 2 - let out line
				winch2.set(-.5);
				SmartDashboard.putNumber("winch2 value", winch.get());

			} else {
				winch2.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}

			/* Shooter Pneumatics Lifter */ 
			if (xbox2.getRawButton(6)) { 				// RIGHT BUMPER double solenoid Lift Shooter
				doublesol.set(DoubleSolenoid.Value.kForward);

			} else if (xbox2.getRawButton(5)) { 		// LEFT BUMPER double solenoid Lower Shooter
				doublesol.set(DoubleSolenoid.Value.kReverse);
			}
			
			/* Shooter Pneumatics Pusher */
			if (xbox2.getRawButton(9)) { 				// THUMBSTICK BUTTON DOWN Ball Pusher Retractor
				doublesol2.set(DoubleSolenoid.Value.kForward);

			} else if (xbox2.getRawButton(10)) { 		// THUMBSTICK BUTTON DOWN Ball Pusher
				doublesol2.set(DoubleSolenoid.Value.kReverse);
			}
			
			/* Shooter Wheels */
			if (xbox2.getRawAxis(2) > 0.0) {			// RIGHT BUMPER
				shooter.set(xbox2.getRawAxis(2));
				SmartDashboard.putNumber("shooter value", shooter.get());

			} else if (xbox2.getRawAxis(3) > 0.0) {		// LEFT BUMPER
				shooter.set(-xbox2.getRawAxis(3));
				SmartDashboard.putNumber("shooter value", shooter.get());

			} else {
				shooter.stopMotor();
				SmartDashboard.putNumber("shooter value", shooter.get());

			}

			// CURRENTLY ARM NO LONGER ATTACHED 2016-02-16			
			//		if (xbox2.getRawButton(4)) { 				// Y BUTTON arm motor UP
			//			arm.set(.5);
			//			SmartDashboard.putNumber("arm value", arm.get());
			//		} else if (xbox2.getRawButton(3)) { 		// X BUTTON arm motor DOWN
			//			arm.set(-.5);
			//			SmartDashboard.putNumber("arm value", arm.get());
			//		} else {
			//			arm.stopMotor();
			//			SmartDashboard.putNumber("arm value", arm.get());
			//		}
			//
			
			// START of CAMERA CODE
			  NIVision.IMAQdxStartAcquisition(session);

		        /**
		         * grab an image, draw the circle, and provide it for the camera server
		         * which will in turn send it to the dashboard.
		         */
		        NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);

		        while (isOperatorControl() && isEnabled()) {

		            NIVision.IMAQdxGrab(session, frame, 1);
		            NIVision.imaqDrawShapeOnImage(frame, frame, rect,
		                    DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
		            
		            CameraServer.getInstance().setImage(frame);

		            /** robot code here! **/
		            Timer.delay(0.005);		// wait for a motor update time
		        }
		        NIVision.IMAQdxStopAcquisition(session);
			// END of CAMERA CODE
		}
		
		
	}

	public void testPeriodic() {

	}

}

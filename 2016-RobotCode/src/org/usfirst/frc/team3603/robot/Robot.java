
/****************************************
 * 
 *  MANTIS Iterative Based JAVA for 2016 FRC Season
 *	@author CyberCoyotes
 *
 ****************************************/

package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	/* Assigns speed controllers to names and PWM numbers */

	Joystick xbox = new Joystick(0);
	Victor left = new Victor(0); // PWM channel 0 for left drive train
	Victor right = new Victor(1); // PWN channel 1 for right motor drive train
	RobotDrive maindrive = new RobotDrive(left, right);
	Joystick xbox2 = new Joystick(1);
	Talon lift = new Talon(2); // scissor lift screw drive motor
	Talon shooter = new Talon(3); // shooter wheels
	Victor winch = new Victor(4); // CIM winch 1 motor
	Victor winch2 = new Victor(5); // CIM winch 2 motor
	Compressor single = new Compressor();
	DoubleSolenoid doublesol = new DoubleSolenoid(1, 2); // lifter
	DoubleSolenoid doublesol2 = new DoubleSolenoid(6, 7); // pusher
	DoubleSolenoid doublesol3 = new DoubleSolenoid(4, 5); // arm
	AnalogInput ultrasonic = new AnalogInput(0);
	AnalogGyro gyro = new AnalogGyro(1);
	ADXRS450_Gyro fancy = new ADXRS450_Gyro();

	CameraServer camera = CameraServer.getInstance();

	Timer Timer = new Timer();
	
	boolean able = true;

	public void robotInit() {
		camera.setQuality(20);
		camera.startAutomaticCapture("cam0");
		Timer.reset();
		single.start();
		Timer.start();

		/********************************
		 * THIS IS A TEST FOR ROBOREALM *
		 ********************************/
		
		/*
		NetworkTable server = NetworkTable.getTable("SmartDashboard");
		try
		{
				System.out.println(server.getNumber("IMAGE_COUNT", 0.0));
		}
		catch (TableKeyNotDefinedException ex)
		{
		}
		*/
	}

	public void autonomousInit() {
		Timer.reset();
		Timer.start();
	}
	int one = 2;
	public void autonomousPeriodic() {
		while (isAutonomous() && able == true && one ==1) {
			doublesol2.set(DoubleSolenoid.Value.kForward);
			Timer.reset();
			while(Timer.get() < 3.0) { //drive forwards for 3 seconds
				maindrive.tankDrive(0.6, 0.6);
			}
			while(fancy.getAngle() > -90) { //Turn to -90 deg.
				maindrive.tankDrive(-0.6, 0.6);
			}
			while(fancy.getAngle() < 90) { //Turn to 90 deg.
				maindrive.tankDrive(0.6, -0.6);
			}
			while(fancy.getAngle() > 0) { //Turn to 0 deg.
				maindrive.tankDrive(-0.6, 0.6);
			}
			Timer.reset();
			while(Timer.get() < 1.5) {
				maindrive.tankDrive(-0.6, -0.6);
			}
			while(fancy.getAngle() < 360) { //Turn to 450 deg.
				maindrive.tankDrive(0.6, -0.6);
			}
			fancy.reset();
			while(fancy.getAngle() < 90) { //Turn to 450 deg.
				maindrive.tankDrive(0.6, -0.6);
			}
			Timer.reset();
			while(Timer.get() < 2.0) { //Drive backwards for 2 seconds
				maindrive.tankDrive(-0.6, -0.6);
				doublesol.set(DoubleSolenoid.Value.kReverse);
				shooter.set(-0.9);
			}
			while(Timer.get()<4.0) {
				shooter.set(-0.9);
			}
			//Shoot
			shooter.set(-0.9);
			doublesol2.set(DoubleSolenoid.Value.kReverse);
			shooter.set(0);
			able = false;
			while(Timer.get()<0.1) {
				
			}
			doublesol2.set(DoubleSolenoid.Value.kForward);
			
			
			/*
			double distanceinches = ultrasonic.getValue() / 9.766;
			//Drive through low bar
			while(Timer.get() < 4.0) {
				maindrive.tankDrive(.55, .55);
			}
			//Setting general front-to-back
			while(Timer.get() < 6.0 && distanceinches > 115) {
				distanceinches = ultrasonic.getValue() / 9.766;
				maindrive.tankDrive(.55, .55);
				doublesol.set(DoubleSolenoid.Value.kForward);
			}
			doublesol.set(DoubleSolenoid.Value.kForward);
			//Turn right 90
			while(Timer.get() < 8.0 && gyro.getAngle() < 90) {
				doublesol.set(DoubleSolenoid.Value.kForward);
				maindrive.tankDrive(.55, -.55);
				shooter.set(.9);
			}
			//Adjust side-to-side
			distanceinches = ultrasonic.getValue() / 9.766;
			while(Timer.get() < 11.0 && distanceinches > 150) {
				maindrive.tankDrive(.55, .55);
				shooter.set(.9);

			}
			//Line up for shooting and fire
			shooter.set(-.9);
			while(Timer.get() < 14.9) {
				if (gyro.getAngle() > 360 || gyro.getAngle() < -360) {
					gyro.reset();
				}
				distanceinches = ultrasonic.getValue() / 9.766;
				if (gyro.getAngle() < -2.5 || gyro.getAngle() > 180) {
					maindrive.tankDrive(.55, -.55); // Turn right
					shooter.set(-.9);
				} else if (gyro.getAngle() > 2.5 || gyro.getAngle() < -180) {
					maindrive.tankDrive(-.55, .55); // Turn left
					shooter.set(-.9);

				} else {
					distanceinches = ultrasonic.getValue() / 9.766;
					shooter.set(-.9);

					if (distanceinches < 115) {
						maindrive.tankDrive(-.55, -.55);
						shooter.set(-.9);

					}
					if (distanceinches > 115) {
						//Begin shooting
						shooter.set(-.9);
						doublesol.set(DoubleSolenoid.Value.kReverse);
					}
				}
			}
			
			
			*/
			
			
			
			
			
			
			// OTHER AUTONOMOS CODE for it to go over low stuff
			// while(Timer.get() < 4.0) {
			// doublesol3.set(DoubleSolenoid.Value.kForward);
			// doublesol.set(DoubleSolenoid.Value.kForward);

			// }
			// while (Timer.get() < 9.0) {
			// maindrive.tankDrive(.8, .8);
			// }
			// maindrive.tankDrive(0, 0);
			// Timer.stop();
			
			
			//Timer no shoot auton- in progress
			/**double distanceinches = ultrasonic.getValue() / 9.766;
			SmartDashboard.putNumber("Ultrasonic in Auton1", distanceinches);
			while (Timer.get() < 3.0) {
				doublesol3.set(DoubleSolenoid.Value.kReverse);
				distanceinches = ultrasonic.getValue() / 9.766;
				SmartDashboard.putNumber("Ultrasonic in Auton2", distanceinches);

			}
			while (Timer.get() < 6.0) {
				maindrive.tankDrive(.7, .7);
				distanceinches = ultrasonic.getValue() / 9.766;
				SmartDashboard.putNumber("Ultrasonic in Auton3", distanceinches);

			}
			while (Timer.get() < 8.0) {
				maindrive.tankDrive(.9, .9);
				distanceinches = ultrasonic.getValue() / 9.766;
				SmartDashboard.putNumber("Ultrasonic in Auton4", distanceinches);

			}

			distanceinches = ultrasonic.getValue() / 9.766;
			SmartDashboard.putNumber("Ultrasonic in Auton5", distanceinches);
			maindrive.tankDrive(0, 0);
			Timer.stop();
			*/
			
			//Timer shooting auton - in progress
			/**
			 * double distanceinches = ultrasonic.getValue() / 9.766;
			 * 
			 * while (Timer.get() < 2.5) {
			 * doublesol3.set(DoubleSolenoid.Value.kReverse); distanceinches =
			 * ultrasonic.getValue() / 9.766; SmartDashboard.putNumber(
			 * "Ultrasonic in Auton0", distanceinches);
			 * 
			 * } while (Timer.get() < 5.5) { maindrive.tankDrive(.85, .85);
			 * doublesol3.set(DoubleSolenoid.Value.kReverse); distanceinches =
			 * ultrasonic.getValue() / 9.766; SmartDashboard.putNumber(
			 * "Ultrasonic in Auton1", distanceinches);
			 * 
			 * } while (Timer.get() < 5.6) {
			 * doublesol.set(DoubleSolenoid.Value.kForward); distanceinches =
			 * ultrasonic.getValue() / 9.766; SmartDashboard.putNumber(
			 * "Ultrasonic in Auton2", distanceinches);
			 * 
			 * } while (Timer.get() < 6.6) { maindrive.tankDrive(.85, .85);
			 * distanceinches = ultrasonic.getValue() / 9.766;
			 * SmartDashboard.putNumber("Ultrasonic in Auton3", distanceinches);
			 * 
			 * }
			 * 
			 * while (Timer.get() < 7.5) { maindrive.tankDrive(.5, -.5);
			 * shooter.set(-.9); doublesol.set(DoubleSolenoid.Value.kForward);
			 * distanceinches = ultrasonic.getValue() / 9.766;
			 * SmartDashboard.putNumber("Ultrasonic in Auton4", distanceinches);
			 * 
			 * } maindrive.tankDrive(0, 0);
			 * doublesol.set(DoubleSolenoid.Value.kForward);
			 * 
			 * while (Timer.get() < 8.5) { maindrive.tankDrive(.5, .5);
			 * doublesol.set(DoubleSolenoid.Value.kForward); distanceinches =
			 * ultrasonic.getValue() / 9.766; SmartDashboard.putNumber(
			 * "Ultrasonic in Auton5", distanceinches);
			 * 
			 * } while (Timer.get() < 9.5) {
			 * doublesol.set(DoubleSolenoid.Value.kForward);
			 * 
			 * } while (Timer.get() < 12.0) {
			 * doublesol2.set(DoubleSolenoid.Value.kReverse); distanceinches =
			 * ultrasonic.getValue() / 9.766; SmartDashboard.putNumber(
			 * "Ultrasonic in Auton6", distanceinches);
			 * 
			 * } while (Timer.get() < 14.0) { shooter.set(-.9);
			 * doublesol2.set(DoubleSolenoid.Value.kReverse);
			 * 
			 * } distanceinches = ultrasonic.getValue() / 9.766;
			 * SmartDashboard.putNumber("Ultrasonic in Auton7", distanceinches);
			 * maindrive.tankDrive(0, 0); Timer.stop();
			 */

		
		}

	}

	public void teleopPeriodic() {
		while (isOperatorControl() && isEnabled()) {
			double magnitude1 = xbox.getRawAxis(1);
			double magnitude2 = xbox.getRawAxis(5);
			double all = magnitude1 + magnitude2;
			if(all < -.25 || all > .25) {
				maindrive.tankDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(5));
			}

			double distanceinches = ultrasonic.getValue() / 9.766;
			SmartDashboard.putNumber("distance in inches", distanceinches);

			/****************************************
			 * BLACK XBOX CONTROLLER DRIVER + SHOOT BALL
			 ***************************************/
			SmartDashboard.putNumber("Timer value", Timer.get());
			
			if(xbox.getRawButton(1)) {
				maindrive.tankDrive(.75, .75);
			}
			if(xbox.getRawButton(2)) {
				maindrive.tankDrive(-.75, -.75);
			}
			if(xbox.getRawButton(3)) {
				maindrive.tankDrive(.75, -.75);
			}
			if(xbox.getRawButton(4)) {
				maindrive.tankDrive(-.75, .75);
			}
			//Self-adjust shooting with button five- Left Bumper
			while (xbox.getRawButton(5)) {
				if (gyro.getAngle() > 360 || gyro.getAngle() < -360) {
					gyro.reset();
				}
				distanceinches = ultrasonic.getValue() / 9.766;
				if (gyro.getAngle() < -2.5 || gyro.getAngle() > 180) {
					maindrive.tankDrive(.55, -.55); // Turn right if too far left
				} else if (gyro.getAngle() > 2.5 || gyro.getAngle() < -180) {
					maindrive.tankDrive(-.55, .55); // Turn left if too far right
				} else {
					distanceinches = ultrasonic.getValue() / 9.766;
					if (distanceinches < 115) {
						maindrive.tankDrive(-.55, -.55);
					}
					if (distanceinches > 115) {
						maindrive.tankDrive(.25, .25);
						//Pause for one second
						try {
							Thread.sleep(1000);
						} catch (InterruptedException e1) {
							e1.printStackTrace();
						}
						//Ready the shooters
						for(int x = 1; x<=1000; x++) {
							shooter.set(-.9);
						}
						shooter.set(-.9);
						//Shoot
						doublesol2.set(DoubleSolenoid.Value.kReverse);
						//Pause for 0.5 seconds
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
						doublesol2.set(DoubleSolenoid.Value.kForward);
					}
				}
			}
			/* Shooter Pneumatics Pusher */
			if (xbox.getRawButton(6)) { // RIGHT BUMPER
				doublesol2.set(DoubleSolenoid.Value.kReverse);
			} else {
				doublesol2.set(DoubleSolenoid.Value.kForward);
			}

			if (xbox.getRawButton(7)) { // BACK BUTTON 7 Drive backwards at full speed
				maindrive.tankDrive(-1.0, -1.0);
			}
			if (xbox.getRawButton(8)) { // START BUTTON 8 Drive forward at full speed
				maindrive.tankDrive(1.0, 1.0);
			}
			if(xbox.getRawButton(10)) {   // manual gyro calibration
				gyro.calibrate();
			}
			if (xbox.getRawAxis(2) > 0.0) {
				//TRIGGER Left- Drive right side forward with variabe speed at axis command
				maindrive.tankDrive(xbox.getRawAxis(2), 0);
				if (xbox.getRawAxis(3) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}
			if (xbox.getRawAxis(3) > 0.0) {
				//TRIGGER RIGHT- Drive right side forward with variabe speed at axis command
				maindrive.tankDrive(0, xbox.getRawAxis(3));
				if (xbox.getRawAxis(2) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}


			/****************************************
			 * AFTERGLOW XBOX CONTROLLER MANIPULATOR
			 ***************************************/

			if (xbox2.getRawButton(1)) { // A BUTTON
				lift.set(-.75); // Scissor Lift Motor DOWN

			} else if (xbox2.getRawButton(2)) { // B BUTTON Scissor Lift Motor
				lift.set(.5); // Scissor Lift Motor UP

			} else {
				lift.stopMotor();

			}

			/* Arm DoubleSolenoid */
			if (xbox2.getRawButton(3)) { // Y BUTTON
				doublesol3.set(DoubleSolenoid.Value.kForward); // should raise
																// arm - NEEDS
																// VERIFICATION
			} else if (xbox2.getRawButton(4)) { // X BUTTON
				doublesol3.set(DoubleSolenoid.Value.kReverse); // should lower
																// arm - NEEDS
																// VERIFICATION
			}
			// might need to add else statement
			if (xbox2.getRawButton(8)) { // START BUTTON Winch
				winch.set(1.0); // CIM 1 Pull Robot Up
				winch2.set(1.0); // CIM 2 Pull Robot Up - ALWAYS KEEP SAME VALUE
									// AS CIM 1 for Button

			} else if (xbox2.getRawButton(7)) { // BACK BUTTON Winch
				winch.set(-1.0); // CIM 1 Let out line
				winch2.set(-1.0); // CIM 2 Let out line - ALWAYS KEEP SAME VALUE
									// as CIM 1 for Button
			} else {
				winch.stopMotor();
				winch2.stopMotor();

			}

			/* Shooter Pneumatics Lifter */
			if (xbox2.getRawButton(6)) { // RIGHT BUMPER double solenoid Lift
											// Shooter
				doublesol.set(DoubleSolenoid.Value.kForward);

			} else if (xbox2.getRawButton(5)) { // LEFT BUMPER double solenoid
												// Lower Shooter
				doublesol.set(DoubleSolenoid.Value.kReverse);
			}

			if (xbox2.getRawAxis(2) > 0.0) {
				shooter.set(.85);

			} else if (xbox2.getRawAxis(3) > 0.0) {
				shooter.set(-1.0);
			} else {
				shooter.stopMotor();
			}
			if (gyro.getAngle() > 360 || gyro.getAngle() < -360) {
				gyro.reset();
			}
		}
	}

	public void testPeriodic() {}
}


//Button 1 is A
//Button 2 is B
//Button 3 is X
//Button 4 is Y
//Button 5 is left bumper
//Button 6 is right bumper
//Button 7 is select
//Button 8 is start
//Button 9 is left stick
//Button 10 is right stick
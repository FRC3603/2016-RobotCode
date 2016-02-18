
package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// 2016 CyberCoyotes Main JAVA Code

public class Robot extends IterativeRobot {
	Joystick xbox = new Joystick(0);
	Victor left = new Victor(0); // pwm channel 0 for left drive train
	Victor right = new Victor(1); // pwm channel 1 for right motor drive train
	RobotDrive maindrive = new RobotDrive(left, right);
	Joystick xbox2 = new Joystick(1);
	Talon lift = new Talon(2);
	Talon shooter = new Talon(3);
	Talon winch = new Talon(4);
	Victor arm = new Victor(5);
	Compressor single = new Compressor();
	Solenoid singlesol = new Solenoid(0);
	DoubleSolenoid doublesol = new DoubleSolenoid(1, 2);
	Timer Timer = new Timer();
	

	public void robotInit() {
		single.start();
		Timer.start();

	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {
		while (isOperatorControl() && isEnabled()) {
			// XBOX 1 DRIVER CODE
			
			SmartDashboard.putNumber("Timer value", Timer.get());
			

			double xboxmagnitude0 = xbox.getRawAxis(0);
			double xboxmagnitude1 = xbox.getRawAxis(1);
			double xboxmagnitude5 = xbox.getRawAxis(5);

			double all = xboxmagnitude0 + xboxmagnitude1 + xboxmagnitude5;

			if (all > .2 || all < -.2) { // ||
				maindrive.tankDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(5));
			}
			SmartDashboard.putNumber("Timer value", Timer.get());

			if (xbox.getRawButton(1)) { // backwards drive
				maindrive.tankDrive(-.75, -.75);

			}
			if (xbox.getRawButton(4)) { // forward drive
				maindrive.tankDrive(.75, .75);

			}
			if (xbox.getRawButton(2)) { // turn right
				maindrive.tankDrive(.5, -.5);

			}

			if (xbox.getRawButton(3)) { // turn left
				maindrive.tankDrive(-.5, .5);

			}

			if (xbox.getRawButton(6)) { // turn right faster
				maindrive.tankDrive(1.0, -1.0);
			}
			
			if (xbox.getRawButton(5)) { // turn left faster
				maindrive.tankDrive(-1.0, 1.0);
			}
			
	
			
			if (xbox.getRawAxis(2) > 0.0) { // turn left at axis command
				maindrive.tankDrive(0, .5);
			}
			

			// XBOX 2 SHOOTER CODE

			// Lift EXPERIMENTAL CODE
			if (xbox2.getRawButton(2)) {
				lift.set(.5);
				SmartDashboard.putNumber("lift value", lift.get());
			} else if (xbox2.getRawButton(3)) {
				lift.set(-1.0);
				SmartDashboard.putNumber("lift value", lift.get());

			} else {
				lift.stopMotor();
				SmartDashboard.putNumber("lift value", lift.get());

			}

			/*
			 * ORIGINAL if (xbox2.getRawButton(2)) { // lift positive value
			 * lift.set(.5); }
			 */

			/**
			 * if (xbox2.getRawAxis(2) > 0) { // shooter negative value
			 * shooter.set(-xbox.getRawAxis(2)); }
			 * 
			 * if (xbox2.getRawAxis(3) > 0) { // shooter positive value
			 * shooter.set(xbox.getRawAxis(3)); }
			 */

			if (xbox2.getRawButton(8)) { // winch positive value
				winch.set(.2);
				SmartDashboard.putNumber("winch value", winch.get());
			} else if (xbox2.getRawButton(7)) {
				winch.set(-.2);
				SmartDashboard.putNumber("winch value", winch.get());

			} else {
				winch.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}

			/**
			 * if (xbox2.getRawButton(1)) { // arm positive value arm.set(.75);
			 * } if (xbox2.getRawButton(4)) { // arm negative arm arm.set(-.75);
			 * }
			 */
			if (xbox2.getRawButton(6)) { // double solenoid forward
				doublesol.set(DoubleSolenoid.Value.kForward);
			} else if (xbox2.getRawButton(5)) { // double solenoid reverse
				doublesol.set(DoubleSolenoid.Value.kReverse);
			}
			if (xbox2.getRawButton(10)) {
				singlesol.set(true);
			} else if (xbox2.getRawButton(9)) {
				singlesol.set(false);
			}
			if (xbox2.getRawAxis(2) > 0.0) {
				shooter.set(xbox2.getRawAxis(2));
				SmartDashboard.putNumber("shooter value", shooter.get());
			} else if (xbox2.getRawAxis(3) > 0.0) {
				shooter.set(-xbox2.getRawAxis(3));
				SmartDashboard.putNumber("shooter value", shooter.get());

			} else {
				shooter.stopMotor();
				SmartDashboard.putNumber("shooter value", shooter.get());

			}
			SmartDashboard.putNumber("axis two", xbox2.getRawAxis(2));
			SmartDashboard.putNumber("axis three", xbox2.getRawAxis(3));

			// if (xbox2.getRawAxis(1) > 0.2) { // axis arm movement
			// arm.set(xbox2.getRawAxis(1));

			if (xbox2.getRawButton(1)) { // shooter axis movement
				arm.set(.5);
				SmartDashboard.putNumber("Shooter value", shooter.get());
			} else if (xbox2.getRawButton(4)) {
				arm.set(-.5);
				SmartDashboard.putNumber("Shooter value", shooter.get());

			} else {
				arm.stopMotor();
				SmartDashboard.putNumber("Shooter value", shooter.get());

			}

		}
	}

	public void testPeriodic() {

	}

}


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
	Joystick xbox = new Joystick(0); // XBOX controller for driving - USB 0
	Victor left = new Victor(0); // pwm channel 0 for left drive train
	Victor right = new Victor(1); // pwm channel 1 for right motor drive train
	RobotDrive maindrive = new RobotDrive(left, right);
	Joystick xbox2 = new Joystick(1); // XBOX controller for manipulating - USB
										// 1
	Talon lift = new Talon(2); // scissor lift screw drive motor
	Talon shooter = new Talon(3); // shooter wheels
	
	Victor winch = new Victor(4); // winch motor
	Victor winch2 = new Victor(5); // winch motor
	
//	Victor arm = new Victor(5); // arm manipulator motor
	
	Compressor single = new Compressor();
	DoubleSolenoid doublesol2 = new DoubleSolenoid(5,6);
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
			double xboxmagnitude1 = xbox.getRawAxis(1);
			double xboxmagnitude5 = xbox.getRawAxis(5);

			double all = xboxmagnitude1 + xboxmagnitude5;

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
				maindrive.tankDrive(xbox.getRawAxis(2), 0);
				if (xbox.getRawAxis(3) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}
			if (xbox.getRawAxis(3) > 0.0) {
				maindrive.tankDrive(0, xbox.getRawAxis(3));
				if (xbox.getRawAxis(2) > 0.0) {
					maindrive.tankDrive(1.0, 1.0);
				}
			}
			
			// XBOX 2 SHOOTER CODE

			if (xbox2.getRawButton(2)) {
				lift.set(.75); // Scissor Lift Motor UP
				SmartDashboard.putNumber("lift value", lift.get());
			} else if (xbox2.getRawButton(1)) {
				lift.set(-1.0); // Scissor Lift Motor DOWN
				SmartDashboard.putNumber("lift value", lift.get());
			} else {
				lift.stopMotor();
				SmartDashboard.putNumber("lift value", lift.get());

			}
			
			// FIRST WINCH CIM
			if (xbox2.getRawButton(8)) { // winch motor UP
				winch.set(.5);
				SmartDashboard.putNumber("winch value", winch.get());
			} else if (xbox2.getRawButton(7)) { // winch motor DOWN
				winch.set(-.5);
				SmartDashboard.putNumber("winch value", winch.get());

			} else {
				winch.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}
			
			// SECOND WINCH CIM
			if (xbox2.getRawButton(8)) { // winch motor UP
				winch2.set(.5);
				SmartDashboard.putNumber("winch value", winch.get());
			} else if (xbox2.getRawButton(7)) { // winch motor DOWN
				winch2.set(-.5);
				SmartDashboard.putNumber("winch value", winch.get());

			} else {
				winch2.stopMotor();
				SmartDashboard.putNumber("winch value", winch.get());

			}

			if (xbox2.getRawButton(6)) { // double solenoid forward
				doublesol.set(DoubleSolenoid.Value.kForward);
			} else if (xbox2.getRawButton(5)) { // double solenoid reverse
				doublesol.set(DoubleSolenoid.Value.kReverse);
			}
			if (xbox2.getRawButton(9)) { // double solenoid forward
				doublesol2.set(DoubleSolenoid.Value.kForward);
			} else if (xbox2.getRawButton(10)) { // double solenoid reverse
				doublesol2.set(DoubleSolenoid.Value.kReverse);
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
			
	// ARM IS NO LONGER ATTACHED		
	//		if (xbox2.getRawButton(4)) { // arm motor UP
	//			arm.set(.5);
	//			SmartDashboard.putNumber("arm value", arm.get());
	//		} else if (xbox2.getRawButton(3)) { // arm motor DOWN
	//			arm.set(-.5);
	//			SmartDashboard.putNumber("arm value", arm.get());
	//		} else {
	//			arm.stopMotor();
	//			SmartDashboard.putNumber("arm value", arm.get());
	//		}

		}
	}

	public void testPeriodic() {

	}

}

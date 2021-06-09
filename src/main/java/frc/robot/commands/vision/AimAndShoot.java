// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AimAndShoot extends CommandBase {
	boolean firstRun;
	Timer timer;
	PolynomialFunction polynomialFunction = new PolynomialFunction();
	PIDController pidController;

	Accumulator accumulator;
	DriveTrain driveTrain;
	Hood hood;
	Kicker kicker;
	Limelight limelight;
	Shooter shooter;

	/** Creates a new AimAndShoot. */
	public AimAndShoot(Accumulator a, DriveTrain dt, Hood h, Kicker k, Limelight l, Shooter s) {
		accumulator = a;
		driveTrain = dt;
		hood = h;
		kicker = k;
		limelight = l;
		shooter = s;

		timer = new Timer();
		pidController = new PIDController(0.1, 0, 0.015);
		addRequirements(accumulator, driveTrain, hood, kicker, limelight, shooter);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// double[] coefficients = {
		// -947.289, 49.8499, -1.05574, 0.0125854, -0.0000932031, 4.4594e-7, -1.382e-9, 2.6801e-12, -2.9558e-15,
		// 1.4143e-18
		// };
		double[] coefficients = { 10.8025, 0.892735, -0.00520211, 0.0000134909, -1.3041e-8 };
		shooter.setSetpoint(Constants.ShooterConst.SHOOTER_TARGET_SPEED);
		firstRun = true;
		if (limelight.validTarget()) {
			hood.setHood(polynomialFunction.polynomailFunction(
					limelight.targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT), coefficients));
			SmartDashboard.putNumber("Equation Output", polynomialFunction.polynomailFunction(
					limelight.targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT), coefficients));
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (limelight.validTarget()) {
			driveTrain.arcadeDrive(0, pidController.calculate(limelight.xOffset(), 0));
		}
		else {
			driveTrain.stop();
		}
		if (shooter.atSetpoint(Constants.ShooterConst.SHOOTER_TARGET_SPEED)) {
			OperatorInput.driverJoystick.setRumble(RumbleType.kRightRumble, 1);
			if (hood.atSetpoint() && (timer.get() >= Constants.ShooterConst.SHOOTER_DELAY || firstRun)) {
				hood.stop();
				kicker.setKicker(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
				accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
				firstRun = false;
				timer.reset();
				timer.start();
			}
		}
		else if (!kicker.ballLoaded()) {
			OperatorInput.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
			kicker.setKicker(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
			accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
		}
		else {
			OperatorInput.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
			kicker.stopKicker();
			accumulator.stopAccumulator();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		OperatorInput.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
		accumulator.stopAccumulator();
		driveTrain.stop();
		hood.disable();
		kicker.stopKicker();
		shooter.stopShooter();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConst;
import frc.robot.Constants.ShooterConst.Characterization;

public class Shooter extends SubsystemBase {
	CANSparkMax shooterMotor1;
	CANSparkMax shooterMotor2;
	CANPIDController shooterPIDController;

	public CANSparkMax shooterSparkMax(int ID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(ID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kCoast);
		return sparkMax;
	}

	/** Creates a new Shooter. */
	public Shooter() {
		shooterMotor1 = shooterSparkMax(ShooterConst.SHOOTER_MOTOR_1_ID, false);
		shooterMotor2 = shooterSparkMax(ShooterConst.SHOOTER_MOTOR_2_ID, false);
		shooterMotor2.follow(shooterMotor1, true);
		shooterPIDController = shooterMotor1.getPIDController();
		shooterPIDController.setP(ShooterConst.PID.P, 0);
		shooterPIDController.setI(ShooterConst.PID.I, 0);
		shooterPIDController.setD(ShooterConst.PID.D, 0);
		shooterMotor1.enableVoltageCompensation(10);
		shooterMotor1.burnFlash();
		shooterMotor2.burnFlash();

		// SmartDashboard.putNumber("Shooter P", ShooterConst.PID.P);
		// SmartDashboard.putNumber("Shooter I", ShooterConst.PID.I);
		// SmartDashboard.putNumber("Shooter D", ShooterConst.PID.D);
	}

	public void setSetpoint(double setpoint) {
		shooterPIDController.setReference(setpoint, ControlType.kVelocity, 0,
				new SimpleMotorFeedforward(Characterization.S, Characterization.V, Characterization.A)
						.calculate(setpoint));
	}

	public void recoverShooter(double setpoint) {
		shooterPIDController.setReference(setpoint, ControlType.kVelocity, 0,
				new SimpleMotorFeedforward(Characterization.S, Characterization.V, Characterization.A)
						.calculate(shooterMotor1.getEncoder().getVelocity(), setpoint));
	}

	public boolean atSetpoint(double setpoint) {
		return (Math.abs(setpoint - shooterMotor1.getEncoder().getVelocity()) <= 20);
	}

	public void stopShooter() {
		shooterMotor1.stopMotor();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Shooter RPM", shooterMotor1.getEncoder().getVelocity());
		SmartDashboard.putBoolean("Shooter At Setpoint", this.atSetpoint(Constants.ShooterConst.SHOOTER_TARGET_SPEED));

		// shooterPIDController.setP(SmartDashboard.getNumber("Shooter P", ShooterConst.PID.P));
		// shooterPIDController.setI(SmartDashboard.getNumber("Shooter I", ShooterConst.PID.I));
		// shooterPIDController.setD(SmartDashboard.getNumber("Shooter D", ShooterConst.PID.D));

		// This method will be called once per scheduler run
	}
}

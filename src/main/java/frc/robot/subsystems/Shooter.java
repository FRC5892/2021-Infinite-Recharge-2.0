// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
		shooterPIDController.setP(ShooterConst.PID.P);
		shooterPIDController.setI(ShooterConst.PID.I);
		shooterPIDController.setD(ShooterConst.PID.D);
		shooterPIDController.setFF(ShooterConst.PID.FF);

		SmartDashboard.putNumber("Shooter P", ShooterConst.PID.P);
		SmartDashboard.putNumber("Shooter I", ShooterConst.PID.I);
		SmartDashboard.putNumber("Shooter D", ShooterConst.PID.D);
		SmartDashboard.putNumber("Shooter FF", ShooterConst.PID.FF);
	}

	public void setSetpoint(double setpoint) {
		shooterPIDController
				.setFF(new SimpleMotorFeedforward(Characterization.S, Characterization.V, Characterization.A)
						.calculate(setpoint) * ShooterConst.SPARK_MAX_PID_CONVERSION);
		shooterPIDController.setReference(setpoint, ControlType.kVelocity);
	}

	public boolean atSetpoint(double setpoint) {
		return (shooterMotor1.getEncoder().getVelocity() <= setpoint + 20
				&& shooterMotor1.getEncoder().getVelocity() >= setpoint - 20);
	}

	public void stopShooter() {
		shooterMotor1.stopMotor();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Shooter RPM", shooterMotor1.getEncoder().getVelocity());
		SmartDashboard.putNumber("Shooter Setpoint RPM", shooterMotor1.get());
		SmartDashboard.putBoolean("Shooter At Setpoint", this.atSetpoint(ShooterConst.SHOOTER_TARGET_SPEED));

		// shooterPIDController.setP(SmartDashboard.getNumber("Shooter P", ShooterConst.PID.P));
		// shooterPIDController.setI(SmartDashboard.getNumber("Shooter I", ShooterConst.PID.I));
		// shooterPIDController.setD(SmartDashboard.getNumber("Shooter D", ShooterConst.PID.D));
		// shooterPIDController.setFF(SmartDashboard.getNumber("Shooter FF", ShooterConst.PID.FF));

		// This method will be called once per scheduler run
	}
}

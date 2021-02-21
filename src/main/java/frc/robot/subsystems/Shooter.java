// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooterMotor1;
  CANSparkMax shooterMotor2;
  CANPIDController shooterPIDController;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor1 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
    shooterMotor2.follow(shooterMotor1, true);
    shooterPIDController = shooterMotor1.getPIDController();
    shooterPIDController.setP(Constants.Shooter.ShooterPID.P);
    shooterPIDController.setI(Constants.Shooter.ShooterPID.I);
    shooterPIDController.setD(Constants.Shooter.ShooterPID.D);
    shooterPIDController.setFF(Constants.Shooter.ShooterPID.FF);
  }

  public void SetSetpoint(double setpoint) {
    shooterPIDController.setReference(setpoint, ControlType.kVelocity);
  }

  public boolean AtSetpoint (double setpoint) {
    if (shooterMotor1.getEncoder().getVelocity() >= setpoint) {
      return true;
    }
    else {
      return false;
    }
  }
  public void StopShooter() {
    shooterMotor1.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

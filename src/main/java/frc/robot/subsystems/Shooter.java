// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Shooter extends PIDSubsystem {
  CANSparkMax shooterMotor1;
  CANSparkMax shooterMotor2;
  CANEncoder shooterMotor1Encoder;
  /** Creates a new Shooter. */
  public Shooter() {
    super(
      // The PIDController used by the subsystem
      new PIDController(0, 0, 0));
      setSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED);
      shooterMotor1 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
      shooterMotor1Encoder = shooterMotor1.getEncoder();
      shooterMotor2 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
      shooterMotor2.follow(shooterMotor1, true);
    }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotor1.pidWrite(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shooterMotor1Encoder.getVelocity();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
  Shooter shooter;
  Kicker kicker;
  Accumulator accumulator;
  /** Creates a new ShootBall. */
  public ShootBall(Shooter s, Kicker k, Accumulator a) {
    shooter = s;
    kicker = k;
    accumulator = a;
    addRequirements(shooter, kicker, accumulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED)) {
      kicker.setKicker(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
      accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
    }
    if (shooter.belowSetPoint(Constants.Shooter.SHOOTER_TARGET_SPEED, Constants.Shooter.SHOOTER_SHOT_DIFFERENCE)) {
      kicker.stopKicker();
      accumulator.stopAccumulator();
    }
    if (!kicker.ballLoaded()) {
      kicker.setKicker(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
      accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    kicker.stopKicker();
    accumulator.stopAccumulator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

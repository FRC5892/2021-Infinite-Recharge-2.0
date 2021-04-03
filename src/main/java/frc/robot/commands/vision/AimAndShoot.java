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
import frc.robot.RobotContainer;
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
    double[] coefficients = {
      -947.289, 49.8499, -1.05574, 0.0125854, -0.0000932031, 4.4594*Math.pow(10, -7), -1.382*Math.pow(10, -9), 2.6801*Math.pow(10, -12), -2.9558*Math.pow(10, -15),
      1.4143*Math.pow(10, -18)
    };
    if (limelight.validTarget()) {
      shooter.setSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED);
      hood.setHood(polynomialFunction.polynomailFunction(limelight.targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT), coefficients));
      SmartDashboard.putNumber("Equation Output", polynomialFunction.polynomailFunction(limelight.targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT), coefficients));
    }
    firstRun = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.validTarget()) {      
      driveTrain.arcadeDrive(0, pidController.calculate(limelight.xOffset(), 0));
      if (shooter.atSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED)) {
        RobotContainer.driverJoystick.setRumble(RumbleType.kRightRumble, 1);
        if (timer.get() >= Constants.Shooter.SHOOTER_DELAY || firstRun) {
          firstRun = !firstRun;
          RobotContainer.driverJoystick.setRumble(RumbleType.kLeftRumble, 1);
          timer.reset();
          timer.start();
          kicker.setKicker(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
          accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_NUDGE_SPEED);
        }
        else {
          RobotContainer.driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
        }
      }
      if (!shooter.atSetpoint(Constants.Shooter.SHOOTER_TARGET_SPEED)) {
        RobotContainer.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
        RobotContainer.driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
        if (!firstRun) {
          shooter.idleFF();
        }
        if (!kicker.ballLoaded()) {
          kicker.setKicker(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
          accumulator.setAccumulator(Constants.Kicker.KICKER_MOTOR_ADVANCE_SPEED);
        }
        else {
          kicker.stopKicker();
          accumulator.stopAccumulator();
        }
      }
    }
    else {
      driveTrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
    RobotContainer.driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
    accumulator.stopAccumulator();
    driveTrain.stop();
    hood.disable();
    kicker.stopKicker();
    shooter.stopShooter();
    shooter.resetFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

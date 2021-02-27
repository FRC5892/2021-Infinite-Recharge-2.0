// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class Aim extends CommandBase {
  private final Limelight limelight;
  private final Double targetX;
  private double steering;
  private DriveTrain driveTrain;
  private double leftSteering;
  private double rightSteering;
  /** Creates a new Aim. */
  public Aim(Limelight l, DriveTrain d) {
    limelight = l;
    driveTrain = d;
    targetX = limelight.getLimelightTable().getEntry("tx").getNumber(0).doubleValue();
    steering = 0;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetX > 1.0) {
      steering = 1*targetX - .05;
    }
    else if (targetX < 1.0) {
      steering = 1*targetX + .05;
    }
    leftSteering += steering;
    rightSteering -= steering;
    driveTrain.tankDrive(leftSteering, rightSteering);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

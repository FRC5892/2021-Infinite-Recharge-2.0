// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightGetInRange extends CommandBase {
  DriveTrain driveTrain;
  Limelight limelight;
  NetworkTable smartDashboardTable;
  /** Creates a new LimelightGetInRange. */
  public LimelightGetInRange(DriveTrain d, Limelight l) {
    driveTrain = d;
    limelight = l;
    smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceAdjust = 0.05 * (smartDashboardTable.getEntry("Set Limelight Distance").getDouble(0) - limelight.targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT));
    driveTrain.arcadeDrive(distanceAdjust, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

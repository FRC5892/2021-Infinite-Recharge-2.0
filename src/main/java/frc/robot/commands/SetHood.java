// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class SetHood extends CommandBase {
  Hood hood;
  NetworkTable smartDashboardTable;
  /** Creates a new SetHood. */
  public SetHood(Hood h) {
    hood = h;
    smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setHood(smartDashboardTable.getEntry("Set Hood Angle").getDouble(0));
    hood.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atDirectionStop() || hood.atSetpoint();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class SetHood extends CommandBase {
  Hood hood;
  /** Creates a new SetHood. */
  public SetHood(Hood h) {
    hood = h;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setSetpoint(1.5);
    hood.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(hood.getSetpoint());
    System.out.println(hood.getMeasurement());
    System.out.println("Bottom Stop " +hood.getBottomStop());
    System.out.println("Top Stop " +hood.getTopStop());
    System.out.println("At Direction Stop" +hood.atDirectionStop())
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.disable();
    System.out.println("Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atDirectionStop();
  }
}

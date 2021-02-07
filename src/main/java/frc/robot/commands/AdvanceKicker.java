// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Kicker;

public class AdvanceKicker extends CommandBase {
  private final Kicker kicker;
  private boolean finish = false;
  Timer timer;
  /** Creates a new AdvanceKicker. */
  public AdvanceKicker(Kicker k) {
    kicker = k;
    addRequirements(kicker);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (kicker.ballLoaded() != true ){
      kicker.startKicker();
    }
    else {
      finish = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kicker.ballLoaded()) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

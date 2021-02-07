// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.intakeToggle.rollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class DislodgeIntake extends CommandBase {
  Intake intake;
  private boolean finish;
  Timer timer;
  Value doubleSolenoidValue;
  /** Creates a new DislodgeBall. */
  public DislodgeIntake(Intake i) {
    finish = false;
    intake = i;
    addRequirements(intake);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    doubleSolenoidValue = intake.getSolenoidValue();
    timer.reset();
    timer.start();
      if (doubleSolenoidValue == Value.kReverse) {
        while(timer.get() < Constants.Intake.DISLODGE_SPIN_REVERSE_TIME){
          intake.setRollersSpeed(Constants.Intake.DISLODGE_ROLLERS_SPEED);
        }
        timer.reset();
        timer.start();
        while(timer.get() < Constants.Intake.DISLODGE_SPIN_EXTEND_TIME){
        intake.setRollersSpeed(-Constants.Intake.DISLODGE_ROLLERS_SPEED);
      }
        finish = true;
      }
      else {
        while(timer.get() < Constants.Intake.DISLODGE_SPIN_RETRACT_TIME){
          intake.setRollersSpeed(-Constants.Intake.DISLODGE_ROLLERS_SPEED);
        }
        finish = true;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.Intake;

public class RunIntakeRollers extends CommandBase {
	Intake intake;
	Accumulator accumulator;

	/** Creates a new RunIntakeRollers. */
	public RunIntakeRollers(Intake i) {
		intake = i;
		addRequirements(intake);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (OperatorInput.driverJoystick.getTriggerAxis(OperatorInput.outtakeHand) != 0) {
			intake.setRollersSpeed(-(OperatorInput.driverJoystick.getTriggerAxis(OperatorInput.outtakeHand)
					* Constants.Intake.ROLLER_SPEED));

		}
		else if (OperatorInput.driverJoystick.getTriggerAxis(OperatorInput.intakeHand) != 0) {
			intake.setRollersSpeed((OperatorInput.driverJoystick.getTriggerAxis(OperatorInput.intakeHand)
					* Constants.Intake.ROLLER_SPEED));
		}
		else {
			intake.stopRollers();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopRollers();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

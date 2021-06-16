// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveAuton extends CommandBase {
	DriveTrain driveTrain;
	Boolean finish;

	/** Creates a new DriveRotations. */
	public DriveAuton(DriveTrain d) {
		driveTrain = d;
		addRequirements(driveTrain);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		driveTrain.resetEncoders();
		System.out.println("Running DriveAuton");
		driveTrain.resetEncoders();
		finish = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Math.abs(driveTrain.getLeftEncoder().getPosition()) <= 1) {
			driveTrain.arcadeDrive(.5, 0);
		}
		else {
			driveTrain.stop();
			finish = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveTrain.stop();
		System.out.println("Stopping DriveAuton");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finish;
	}
}

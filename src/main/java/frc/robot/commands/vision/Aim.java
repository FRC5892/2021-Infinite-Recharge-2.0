// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aim extends PIDCommand {
	DriveTrain driveTrain;
	Limelight limelight;

	/** Creates a new Aim. */
	public Aim(DriveTrain driveTrain, Limelight limelight) {
		super(
				// The controller that the command will use
				new PIDController(0.1, 0, 0.015),
				// This should return the measurement
				() -> limelight.xOffset(),
				// This should return the setpoint (can also be a constant)
				() -> 0,
				// This uses the output
				output -> {
					driveTrain.arcadeDrive(0, output);
					// Use the output here
				});
		// Use addRequirements() here to declare subsystem dependencies.
		// Configure additional PID options by calling `getController` here.
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

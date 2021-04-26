// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutonPath extends SequentialCommandGroup {
	DriveTrain driveTrain;

	/** Creates a new testAutonPath. */
	public TestAutonPath(DriveTrain d) {
		driveTrain = d;
		SequentialCommandGroup ramseteCommand = new RamseteCommandGenerator().ramseteCommandGenerator(driveTrain,
				"paths/TestPath.wpilib.json");
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(new ResetEncoders(driveTrain), ramseteCommand);
	}
}

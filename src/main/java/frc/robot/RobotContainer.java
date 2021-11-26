// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveRotations;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SetHood;
import frc.robot.commands.ShootBall;
import frc.robot.commands.autonomous.CurvedPath;
import frc.robot.commands.autonomous.DriveAndShoot;
import frc.robot.commands.autonomous.Slalom;
import frc.robot.commands.autonomous.TestAutonPath;
import frc.robot.commands.intake.RunAccumulator;
import frc.robot.commands.intake.RunIntakeRollers;
import frc.robot.commands.intake.RunKicker;
import frc.robot.commands.intake.intakeToggle.IntakeToggle;
import frc.robot.commands.vision.AimAndShoot;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.ClimbWinch;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// Declaring drivetrain
	public final DriveTrain driveTrain;
	private final DriveWithJoysticks driveWithJoystick;
	private final DriveRotations driveRotations;

	// declaring intake and intake commands
	private final Intake intake;
	private final IntakeToggle intakeToggle;
	private final RunIntakeRollers runIntakeRollers;

	// declaring accumulator and accumulator commands
	private final Accumulator accumulator;
	private final RunAccumulator runAccumulator;

	// declaring kicker and kicker commands
	private final Kicker kicker;
	private final RunKicker runKicker;

	// declaring shooter
	private final Shooter shooter;
	private final ShootBall shootBall;

	private final Hood hood;
	private final SetHood setHood;

	private final ClimbArm climbArm;
	private final ClimbWinch climbWinch;

	// Auton chooser, see
	// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
	private final SendableChooser<String> autonomousChooser;

	// Declaring compressor
	private final Compressor compressor;

	// Declaring limelight and limelight commands
	private Limelight limelight;
	private AimAndShoot aimAndShoot;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		autonomousChooser = new SendableChooser<>();

		driveTrain = new DriveTrain();
		driveWithJoystick = new DriveWithJoysticks(driveTrain);
		driveWithJoystick.addRequirements(driveTrain);
		driveRotations = new DriveRotations(driveTrain);
		driveTrain.setDefaultCommand(driveWithJoystick); // drive with joysticks by default

		accumulator = new Accumulator();
		runAccumulator = new RunAccumulator(accumulator);
		accumulator.setDefaultCommand(runAccumulator);

		intake = new Intake();
		intakeToggle = new IntakeToggle(intake);
		runIntakeRollers = new RunIntakeRollers(intake);
		intake.setDefaultCommand(runIntakeRollers);

		kicker = new Kicker();
		runKicker = new RunKicker(kicker);
		kicker.setDefaultCommand(runKicker);

		shooter = new Shooter();
		shootBall = new ShootBall(shooter, kicker, accumulator);

		hood = new Hood();
		setHood = new SetHood(hood);

		climbArm = new ClimbArm();
		climbWinch = new ClimbWinch();

		limelight = new Limelight();
		aimAndShoot = new AimAndShoot(accumulator, driveTrain, hood, kicker, limelight, shooter);

		compressor = new Compressor(0);
		autonomousChooser.setDefaultOption("Default Dumb Auton", "defaultDumbAuton");
		autonomousChooser.addOption("None", null);
		autonomousChooser.addOption("Test Path", "testAutonPath");
		autonomousChooser.addOption("Curved Path", "curvedPath");
		autonomousChooser.addOption("Slalom Path", "slalomPath");
		SmartDashboard.putData("Autonomous mode chooser", autonomousChooser);
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by instantiating a
	 * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
	 * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		OperatorInput.intakeToggleButton.whenPressed(intakeToggle);
		OperatorInput.spinShooterButton.whileHeld(shootBall);
		OperatorInput.setHoodButton.whenPressed(setHood);
		OperatorInput.aimAndShootToggle.toggleWhenPressed(aimAndShoot);
		OperatorInput.climbArmToggle.whenPressed(new InstantCommand(climbArm::toggleArm, climbArm));
		OperatorInput.climbArmDown.whenPressed(() -> climbArm.setArmMotor(-1))
				.whenReleased(() -> climbArm.stopArmMotor());
		OperatorInput.climbArmUp.whenPressed(() -> climbArm.setArmMotor(1)).whenReleased(() -> climbArm.stopArmMotor());
		OperatorInput.runWinch.whenPressed(() -> climbWinch.setWinchMotor(-1))
				.whenReleased(() -> climbWinch.setWinchMotor(0));
		OperatorInput.zeroHood.whenPressed(() -> hood.setHood(0));
		// OperatorInput.driveRotationsButton.whenPressed(driveRotations);

		// codriver functions
		OperatorInput.coIntakeToggle.whenPressed(intakeToggle);
		OperatorInput.coClimbArmToggleButton.whenPressed(new InstantCommand(climbArm::toggleArm, climbArm));
		OperatorInput.coClimbArmDown.whenPressed(() -> climbArm.setArmMotor(-1))
				.whenReleased(() -> climbArm.stopArmMotor());
		OperatorInput.coClimbArmUp.whenPressed(() -> climbArm.setArmMotor(1))
				.whenReleased(() -> climbArm.stopArmMotor());
		OperatorInput.coZeroHood.whenPressed(() -> hood.setHood(0));
		OperatorInput.coWinchButton.whenPressed(() -> climbWinch.setWinchMotor(-1))
				.whenReleased(() -> climbWinch.setWinchMotor(0));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		switch (autonomousChooser.getSelected()) {
		case "defaultDumbAuton":
			return new DriveAndShoot(accumulator, driveTrain, hood, kicker, limelight, shooter);
		case "testAutonPath":
			return new TestAutonPath(driveTrain);
		case "curvedPath":
			return new CurvedPath(driveTrain);
		case "slalomPath":
			return new Slalom(driveTrain);
		default:
			return null;
		}
	}
}

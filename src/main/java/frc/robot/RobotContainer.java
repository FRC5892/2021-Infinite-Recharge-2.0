// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveRotations;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightGetInRange;
import frc.robot.commands.SetHood;
import frc.robot.commands.ShootBall;
import frc.robot.commands.autonomous.TestAutonPath;
import frc.robot.commands.intake.RunAccumulator;
import frc.robot.commands.intake.RunIntakeRollers;
import frc.robot.commands.intake.RunKicker;
import frc.robot.commands.intake.intakeToggle.IntakeToggle;
import frc.robot.commands.vision.Aim;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //Auton chooser, see https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
  private final SendableChooser<Command> autonomousChooser;
  //Declaring drivetrain
  private final DriveTrain driveTrain;
  private final DriveWithJoysticks driveWithJoystick;
  private final DriveForwardTimed driveForwardTimed;
  private final DriveRotations driveRotations;
  public static XboxController driverJoystick;

  //declaring intake and intake commands
  private final Intake intake;
  private final IntakeToggle intakeToggle;
  private final RunIntakeRollers runIntakeRollers;
  
  //declaring accumulator and accumulator commands
  private final Accumulator accumulator;
  private final RunAccumulator runAccumulator;

  //declaring kicker and kicker commands
  private final Kicker kicker;
  private final RunKicker runKicker;

  //declaring shooter
  private final Shooter shooter;
  private final ShootBall shootBall;

  private final Hood hood;
  private final SetHood setHood;

  //Declaring compressor
  private final Compressor compressor;

  //Declaring limelight and limelight commands
  private Limelight limelight;
  private Aim aim;
  private LimelightGetInRange limelightGetInRane;

  //Autonomous Commands
  private TestAutonPath testAutonPath;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonomousChooser = new SendableChooser<>();

    driveTrain = new DriveTrain();
    driveWithJoystick = new DriveWithJoysticks(driveTrain);
    driveWithJoystick.addRequirements(driveTrain);
    driveRotations = new DriveRotations(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoystick); //drive with joysticks by default

    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    driverJoystick = new XboxController(Constants.XboxController.JOYSTICK_NUMBER);
    
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

    limelight = new Limelight();
    aim = new Aim(driveTrain, limelight);
    limelightGetInRane = new LimelightGetInRange(driveTrain, limelight);

    testAutonPath = new TestAutonPath(driveTrain);

    autonomousChooser.setDefaultOption("None", null);
    autonomousChooser.addOption("Test Path", testAutonPath);
    autonomousChooser.addOption("Drive Forward", driveForwardTimed);
    
    compressor = new Compressor(0);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeToggleButton = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    intakeToggleButton.whenPressed(intakeToggle);
    JoystickButton spinShooterButton = new JoystickButton(driverJoystick, XboxController.Button.kStart.value);
    spinShooterButton.whileHeld(shootBall);
    JoystickButton setHoodButton = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
    setHoodButton.whileHeld(setHood);
    JoystickButton aimButton = new JoystickButton(driverJoystick, XboxController.Button.kBumperLeft.value);
    aimButton.whileHeld(aim);
    JoystickButton driveRotationsButton = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
    driveRotationsButton.whenPressed(driveRotations);
    JoystickButton rangeButton = new JoystickButton(driverJoystick, XboxController.Button.kBumperRight.value);
    rangeButton.whileHeld(limelightGetInRane);
    // JoystickButton driveRotationsButton = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
    // driveRotationsButton.whenPressed(driveRotations);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    return autonomousChooser.getSelected();
  }
}

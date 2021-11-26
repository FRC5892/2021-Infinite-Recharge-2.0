// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class OperatorInput {
	public static XboxController driverJoystick = new XboxController(0);
	static JoystickButton intakeToggleButton = new JoystickButton(driverJoystick,
			XboxController.Button.kBumperLeft.value);
	static JoystickButton aimAndShootToggle = new JoystickButton(driverJoystick,
			XboxController.Button.kBumperRight.value);
	static JoystickButton climbArmUp = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
	static JoystickButton climbArmDown = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
	static POVButton climbArmToggle = new POVButton(driverJoystick, 180);
	static POVButton runWinch = new POVButton(driverJoystick, 0);
	public static POVButton runAccumulator = new POVButton(driverJoystick, 90);
	public static POVButton runAccumulatorInverted = new POVButton(driverJoystick, 270);
	// JoystickButton driveRotationsButton = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
	public static final Hand intakeHand = Hand.kRight;
	public static final Hand outtakeHand = Hand.kLeft;
	public static final int xSpeedAxis = 1;
	public static final int zRotationAxis = 4;

	static JoystickButton spinShooterButton = new JoystickButton(driverJoystick, XboxController.Button.kStart.value);
	static JoystickButton setHoodButton = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
	static JoystickButton zeroHood = new JoystickButton(driverJoystick, XboxController.Button.kY.value);

	// codriver controls
	public static XboxController codriverJoystick = new XboxController(1);
	static JoystickButton coIntakeToggle = new JoystickButton(codriverJoystick,
			XboxController.Button.kBumperRight.value);
	static JoystickButton coClimbArmToggleButton = new JoystickButton(codriverJoystick,
			XboxController.Button.kBumperLeft.value);
	static POVButton coClimbArmUp = new POVButton(codriverJoystick, 0);
	static POVButton coClimbArmDown = new POVButton(codriverJoystick, 180);
	static JoystickButton coZeroHood = new JoystickButton(codriverJoystick, XboxController.Button.kA.value);
	static JoystickButton coWinchButton = new JoystickButton(codriverJoystick, XboxController.Button.kBack.value);

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class OperatorInput {
	public static XboxController driverJoystick = new XboxController(0);
	static JoystickButton intakeToggleButton = new JoystickButton(driverJoystick,
			XboxController.Button.kBumperLeft.value);
	static JoystickButton spinShooterButton = new JoystickButton(driverJoystick, XboxController.Button.kStart.value);
	static JoystickButton setHoodButton = new JoystickButton(driverJoystick, XboxController.Button.kX.value);
	static JoystickButton aimButton = new JoystickButton(driverJoystick, XboxController.Button.kBumperLeft.value);
	static JoystickButton aimAndShootButton = new JoystickButton(driverJoystick,
			XboxController.Button.kBumperRight.value);
	// JoystickButton driveRotationsButton = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
	// JoystickButton rangeButton = new JoystickButton(driverJoystick, XboxController.Button.kBumperRight.value);
	public static final Hand intakeHand = Hand.kRight;
	public static final Hand outtakeHand = Hand.kLeft;
}

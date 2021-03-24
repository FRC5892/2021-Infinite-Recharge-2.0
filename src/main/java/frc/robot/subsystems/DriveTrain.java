// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.MathUtils;

public class DriveTrain extends SubsystemBase {
  CANSparkMax leftMotor1;
  CANSparkMax leftMotor2;
  CANSparkMax leftMotor3;
  CANSparkMax rightMotor1;
  CANSparkMax rightMotor2;
  CANSparkMax rightMotor3;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  DifferentialDrive drive;
  public CANSparkMax driveCANSparkMax (int ID) {
    CANSparkMax sparkMax = new CANSparkMax(ID, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(false);
    sparkMax.setIdleMode(IdleMode.kBrake);
    return sparkMax;
  }
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMotor1 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR1_ID);
    leftMotor2 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR2_ID);
    leftMotor3 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR3_ID);
    rightMotor1 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR1_ID);
    rightMotor2 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR2_ID);
    rightMotor3 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR3_ID);

    leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks(XboxController controller, double xSpeed, double zRotation) {
    drive.arcadeDrive(MathUtils.signedSquare(controller.getRawAxis(Constants.XboxController.LEFT_X_AXIS))*xSpeed, MathUtils.signedSquare(-controller.getRawAxis(4))*zRotation, false);
    //squares the controller input before the speed factor is multiplied to make the drive smoother
    //false at the end tells the library not to square it because we already did
    //x speed sets speed of forward motion, z speed sets turning speed 
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void stop(){
    drive.stopMotor();
  }
}

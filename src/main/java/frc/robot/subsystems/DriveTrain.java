// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  CANEncoder leftEncoder;
  CANEncoder rightEncoder;
  SpeedControllerGroup leftMotors;
  SpeedControllerGroup rightMotors;
  DifferentialDrive drive;
  AHRS gyro;
  DifferentialDriveOdometry odometry;

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

    leftEncoder = leftMotor1.getEncoder();
    rightEncoder = rightMotor1.getEncoder();
    resetEncoders();

    leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    drive = new DifferentialDrive(leftMotors, rightMotors);

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }
  
  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks(XboxController controller, double xSpeed, double zRotation) {
    drive.arcadeDrive(MathUtils.signedSquare(controller.getRawAxis(Constants.XboxController.LEFT_X_AXIS))*xSpeed, MathUtils.signedSquare(-controller.getRawAxis(4))*zRotation, false);
    //squares the controller input before the speed factor is multiplied to make the drive smoother
    //false at the end tells the library not to square it because we already did
    //x speed sets speed of forward motion, z speed sets turning speed 
  }
  
  public void driveForward(double speed){
    drive.tankDrive(speed, speed);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    leftEncoder.setPositionConversionFactor(Constants.DriveTrain.DriveCharacteristics.ROTATIONS_TO_METERS_CONSTANT);
    leftEncoder.setVelocityConversionFactor(Constants.DriveTrain.DriveCharacteristics.RPM_TO_METERSPS);
    rightEncoder.setPosition(0);
    rightEncoder.setPositionConversionFactor(Constants.DriveTrain.DriveCharacteristics.ROTATIONS_TO_METERS_CONSTANT);
    rightEncoder.setVelocityConversionFactor(Constants.DriveTrain.DriveCharacteristics.RPM_TO_METERSPS);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  //Used in trajectory
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
  
  public void zeroHeading() {
    gyro.reset();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
  }

  public CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return rightEncoder;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  //Used in trajectory
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //Used in trajectory
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(-leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop(){
    drive.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMotor1 = new CANSparkMax(Constants.DriveTrain.LEFT_MOTOR1_ID, MotorType.kBrushless);
    leftMotor1.setInverted(false);
    leftMotor2 = new CANSparkMax(Constants.DriveTrain.LEFT_MOTOR2_ID, MotorType.kBrushless);
    leftMotor2.setInverted(false);
    leftMotor3 = new CANSparkMax(Constants.DriveTrain.LEFT_MOTOR3_ID, MotorType.kBrushless);
    leftMotor3.setInverted(false);
    rightMotor1 = new CANSparkMax(Constants.DriveTrain.RIGHT_MOTOR1_ID, MotorType.kBrushless);
    rightMotor1.setInverted(false);
    rightMotor2 = new CANSparkMax(Constants.DriveTrain.RIGHT_MOTOR2_ID, MotorType.kBrushless);
    rightMotor2.setInverted(false);
    rightMotor3 = new CANSparkMax(Constants.DriveTrain.RIGHT_MOTOR3_ID, MotorType.kBrushless);
    rightMotor3.setInverted(false);

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
    SmartDashboard.putNumber("Left Motor 1", leftMotor1.get());
    SmartDashboard.putNumber("Left Motor 2", leftMotor2.get());
    SmartDashboard.putNumber("Left Motor 3", leftMotor3.get());
    SmartDashboard.putNumber("Left Motors", leftMotors.get());
    SmartDashboard.putNumber("Right Motor 1", rightMotor1.get());
    SmartDashboard.putNumber("Right Motor 2", rightMotor2.get());
    SmartDashboard.putNumber("Right Motor 3", rightMotor3.get());
    SmartDashboard.putNumber("Right Motors", rightMotors.get());
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
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

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    leftEncoder.setPositionConversionFactor(1);
    leftEncoder.setVelocityConversionFactor(1);
    rightEncoder.setPosition(0);
    rightEncoder.setPositionConversionFactor(1);
    rightEncoder.setVelocityConversionFactor(1);
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

  public void tankDriveVolts(double lefttVolts, double rightVolts) {
    leftMotors.setVoltage(lefttVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop(){
    drive.stopMotor();
  }
}

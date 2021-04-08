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
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.autonomous.ResetEncoders;
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
  Field2d field = new Field2d();

  public CANSparkMax driveCANSparkMax (int ID, boolean inverted) {
    CANSparkMax sparkMax = new CANSparkMax(ID, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(inverted);
    sparkMax.setIdleMode(IdleMode.kBrake);
    return sparkMax;
  }
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMotor1 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR1_ID, false);
    leftMotor2 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR2_ID, false);
    leftMotor3 = driveCANSparkMax(Constants.DriveTrain.LEFT_MOTOR3_ID, false);
    rightMotor1 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR1_ID, false);
    rightMotor2 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR2_ID, false);
    rightMotor3 = driveCANSparkMax(Constants.DriveTrain.RIGHT_MOTOR3_ID, false);

    leftEncoder = leftMotor1.getEncoder();
    rightEncoder = rightMotor1.getEncoder();
    resetEncoders();

    leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setRightSideInverted(true);

    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));
    SmartDashboard.putData("Reset Encoders", new ResetEncoders(this));
  }
  
  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getGyroAngle()), -leftEncoder.getPosition(), rightEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Average", getAverageEncoderDistance());
    SmartDashboard.putNumber("Left Encoder", -leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", -leftEncoder.getVelocity());
    SmartDashboard.putNumber("Left", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putData("Field2D", field);
    // System.out.println(getWheelSpeeds());
    // This method will be called once per scheduler run
    field.setRobotPose(odometry.getPoseMeters());
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
  //from ligerbots code
  private double getGyroAngle() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1; // -1 here for unknown reason look in documatation
  }
  //Used in trajectory
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
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
    return odometry.getPoseMeters().getRotation().getDegrees();
  }

  //Used in trajectory
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //Used in trajectory
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(-leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }
  
  public void setMotorSafety(boolean enabled) {
    drive.setSafetyEnabled(enabled);
  }
  
  public void stop(){
    drive.stopMotor();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class GeneratedTrajectory {
    DriveTrain driveTrain;
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveTrain.DriveCharacteristics.TRACK_WIDTH);
    public SequentialCommandGroup generatedTrajectory (DriveTrain d) {
        driveTrain = d;
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveTrain.DriveCharacteristics.VOLTS,
                                           Constants.DriveTrain.DriveCharacteristics.VOLT_SECONDS_PER_METER,
                                           Constants.DriveTrain.DriveCharacteristics.VOLT_SECONDS_SQUARED_PER_METER),
                kinematics,
                10);
    
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(1,
                                 .1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
    
        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );
    
        RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(Constants.DriveTrain.DriveCharacteristics.RAMSETE_B, Constants.DriveTrain.DriveCharacteristics.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                Constants.DriveTrain.DriveCharacteristics.VOLTS,
                Constants.DriveTrain.DriveCharacteristics.VOLT_SECONDS_PER_METER,
                Constants.DriveTrain.DriveCharacteristics.VOLT_SECONDS_SQUARED_PER_METER),
                kinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.DriveTrain.DriveCharacteristics.P, 0, 0),
            new PIDController(Constants.DriveTrain.DriveCharacteristics.P, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);

    
        // Reset odometry to the starting pose of the trajectory.
        driveTrain.resetOdometry(trajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
//import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class RamseteCommandGenerator {
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveTrain.DriveCharacteristics.TRACK_WIDTH);
    DriveTrain driveTrain;
    public SequentialCommandGroup ramseteCommandGenerator (DriveTrain d, String trajectoryJSON) {
        Trajectory trajectory = new Trajectory();
        driveTrain = d;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            trajectory = null;
        }
    
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

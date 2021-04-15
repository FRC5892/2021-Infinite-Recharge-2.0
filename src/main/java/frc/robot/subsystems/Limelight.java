// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
	NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
	/** Creates a new LimeLight. */
	private final NetworkTableEntry tv, tx, ty, ta;

	public Limelight() {
		limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
		tv = limelightTable.getEntry("tv");
		tx = limelightTable.getEntry("tx");
		ty = limelightTable.getEntry("ty");
		ta = limelightTable.getEntry("ta");
	}

	public boolean validTarget() {
		return tv.getDouble(0) > 0;
	}

	public double xOffset() {
		return tx.getDouble(0);
	}

	public double yOffset() {
		return ty.getDouble(0);
	}

	public double targetArea() {
		return ta.getDouble(0);
	}

	public double targetDistance(double height) {
		if (!validTarget()) {
			return 0;
		}
		else {
			return (height - Constants.Limelight.LIMELIGHT_MOUNTING_HEIGHT)
					/ (Math.tan(Math.toRadians(yOffset() + Constants.Limelight.LIMELIGHT_MOUNTING_ANGLE)));
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.setDefaultNumber("Set Limelight Distance", 0);
		SmartDashboard.putNumber("Limelight Distance", targetDistance(Constants.Limelight.LIMELIGHT_TARGET_HEIGHT));
		// This method will be called once per scheduler run
	}

	public NetworkTable getLimelightTable() {
		return limelightTable;
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils.PoseEstimator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class DemaciaKinematics extends SwerveDriveKinematics {
    public DemaciaKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
    }

    @Override
    public void copyInto(SwerveModulePosition[] positions, SwerveModulePosition[] output) {
        if (positions.length != output.length) {
            throw new IllegalArgumentException("Inconsistent number of modules!");
        }
        for (int i = 0; i < positions.length; ++i) {
            output[i].distanceMeters = positions[i].distanceMeters;
            output[i].angle = positions[i].angle;
        }
    }

    @Override
    public SwerveModulePosition[] copy(SwerveModulePosition[] positions) {
        var newPositions = new SwerveModulePosition[positions.length];
        for (int i = 0; i < positions.length; ++i) {
            newPositions[i] = positions[i].copy();
        }
        return newPositions;
    }

    

    @Override
    public Twist2d toTwist2d(SwerveModulePosition[] start, SwerveModulePosition[] end) {
        if (start.length != end.length) {
            throw new IllegalArgumentException("Inconsistent number of modules!");
        }
        var newPositions = new SwerveModulePosition[start.length];
        for (int i = 0; i < start.length; i++) {
            newPositions[i] = new SwerveModulePosition(end[i].distanceMeters - start[i].distanceMeters, start[i].angle.plus((end[i].angle.plus(start[i].angle)).div(2)));
        }
        return super.toTwist2d(newPositions);
    }   

}

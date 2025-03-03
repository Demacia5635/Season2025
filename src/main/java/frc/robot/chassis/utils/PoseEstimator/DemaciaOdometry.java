// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class DemaciaOdometry<T> {
    private DemaciaKinematics<T> kinematics;

    private Pose2d pose;

    private Rotation2d gyroOffset;
    private Rotation2d prevAngle;
    private T prevWheelPositions;

    public DemaciaOdometry(Rotation2d gyroAngle,
            T wheelPositions,
            Pose2d initialPoseMeters, DemaciaKinematics<T> kinematics) {
        this.kinematics = kinematics;
        pose = initialPoseMeters;
        gyroOffset = pose.getRotation().minus(gyroAngle);
        prevAngle = pose.getRotation();
        prevWheelPositions = kinematics.copy(wheelPositions);

    }

    public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
        var angle = gyroAngle.plus(gyroOffset);

        var twist = kinematics.toTwist2d(prevWheelPositions, wheelPositions);
        twist.dtheta = angle.minus(prevAngle).getRadians();

        var newPose = m_poseMeters.exp(twist);

        kinematics.copyInto(wheelPositions, prevWheelPositions);
        prevAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);

        return pose;
    }

    private boolean isSkidding(T wheelPosition, T lastWheelPosition){
        
    }
    private boolean isSKidding(){}

    public Pose2d getEstimatedPosition() {
        return pose;
    }
}

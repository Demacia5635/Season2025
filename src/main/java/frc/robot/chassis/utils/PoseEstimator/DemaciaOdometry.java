// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class DemaciaOdometry<T> {
    private double[] odometrySTD = new double[3];
    private DemaciaKinematics<T> kinematics = new DemaciaKinematics<>();

    private Pose2d pose;

    private Rotation2d gyroOffset;
    private Rotation2d prevAngle;
    private T prevWheelPositions;

    public DemaciaOdometry(double[] odometrySTD) {
        this.odometrySTD = odometrySTD;

    }

    public void setOdometrySTD(double[] odometrySTD) {
        this.odometrySTD = odometrySTD;
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

    public Pose2d getEstimatedPosition() {
        return pose;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Paths;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Paths.PathsConstants.*;

/** Add your docs here. */
public class Curve {
    private enum CURVE_STATES {
        FOLLOW, FIX
    }

    private double curveLength;
    private double currentDistance;
    private TrajectoryState pointA;
    private TrajectoryState pointB;
    private Translation2d controlA;
    private Translation2d controlB;

    private Trapezoid driveTrapezoid;
    private Trapezoid rotationTrapezoid;
    private CURVE_STATES currentState;

    private final int SAMPLES = 300;
    private TrajectoryState lastState;
    private Pose2d currentPose;
    private double scaledT;

    public Curve(TrajectoryState pointA, TrajectoryState pointB, Translation2d controlA, Translation2d controlB) {
        this.pointA = pointA;
        this.pointB = pointB;
        this.controlA = controlA;
        this.controlB = controlB;
        this.curveLength = calculateLength();
        this.currentDistance = 0.0;
        this.scaledT = 0.0;
        this.currentPose = Pose2d.kZero;
        this.currentState = CURVE_STATES.FOLLOW;
        this.driveTrapezoid = new Trapezoid(DRIVE_CONSTRAINTS.MAX_VELOCITY, DRIVE_CONSTRAINTS.MAX_ACCEL,
                pointB.getVelocityAtPoint());

        this.rotationTrapezoid = new Trapezoid(ROTATION_CONSTRAINTS.MAX_VELOCITY, ROTATION_CONSTRAINTS.MAX_ACCEL,
                ROTATION_CONSTRAINTS.MAX_JERK, 0);

        this.lastState = TrajectoryState.kEmptyState;
    }

    private void updateCurrentState(Translation2d currentPose) {
        if (getPointOnCurve(currentDistance / curveLength).getDistance(currentPose) < MAX_TRAJECTORY_DISTANCE_THRESHOLD)
            currentState = CURVE_STATES.FOLLOW;
        else
            currentState = CURVE_STATES.FIX;
    }

    public ChassisSpeeds calculate(Pose2d currentPose) {
        this.currentPose = currentPose;
        updateCurrentState(currentPose.getTranslation());
        if (currentState == CURVE_STATES.FOLLOW) {

            updateDistanceTravelled(currentPose);
            scaledT = currentDistance / curveLength;
            Rotation2d wantedAngle = pointB.getRotation();

        }
        else{
            
        }

    }

    private Rotation2d fixVector(double currentT, Translation2d currentPosition) {
        return getPointOnCurve(currentT + ((1 - currentT) / 2)).minus(currentPosition).getAngle();
    }

    private void updateDistanceTravelled(Pose2d currentPose) {
        currentDistance += (lastState.getTranslation().getDistance(currentPose.getTranslation()));
    }

    private Translation2d getPointOnCurve(double t) {
        double x = Math.pow(1 - t, 3) * pointA.getX() +
                3 * Math.pow(1 - t, 2) * t * controlA.getX() +
                3 * (1 - t) * Math.pow(t, 2) * controlB.getX() +
                Math.pow(t, 3) * pointB.getX();

        double y = Math.pow(1 - t, 3) * pointA.getY() +
                3 * Math.pow(1 - t, 2) * t * controlA.getY() +
                3 * (1 - t) * Math.pow(t, 2) * controlB.getY() +
                Math.pow(t, 3) * pointB.getY();

        return new Translation2d(x, y);
    }

    private double calculateLength() {
        double temp = 0.0;
        Translation2d prevPoint = getPointOnCurve(0);
        for (int i = 1; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;
            Translation2d currentPoint = getPointOnCurve(t);
            temp += prevPoint.getDistance(currentPoint);
            prevPoint = currentPoint;
        }
        return temp;
    }

    public boolean isFinishedCurve() {
        return Math.abs(currentDistance - curveLength) <= MAX_POSITION_THRESHOLD
        || currentPose.getTranslation().getDistance(pointB.getTranslation()) <= MAX_POSITION_THRESHOLD;
    }

}

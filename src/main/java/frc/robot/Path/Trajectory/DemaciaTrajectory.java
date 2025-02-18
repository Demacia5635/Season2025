// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static frc.robot.Path.Trajectory.TrajectoryConstants.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.TrajectoryConstants.PathsConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Path.Utils.*;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.utils.LogManager;

/** Add your docs here. */
public class DemaciaTrajectory {
    private ArrayList<Segment> segments;
    private double trajectoryLength;
    private double distanceTraveledOnSegment;
    private ArrayList<PathPoint> points;
    private RoundedPoint[] corners;
    private int segmentIndex;
    private Rotation2d wantedAngle;
    private double distanceLeft;
    Pose2d chassisPose = new Pose2d();
    boolean isAlgae;
    private boolean isAuto;
    double maxVel;
    double accel;

    /*
     * 
     * given points based on blue alliance
     * 
     */
    public DemaciaTrajectory(ArrayList<PathPoint> points, boolean isRed, Rotation2d wantedAngle, Pose2d initialPose,
            boolean isAlgae) {
        this.segments = new ArrayList<Segment>();
        this.trajectoryLength = 0;
        this.distanceTraveledOnSegment = 0;
        this.points = points;
        this.segmentIndex = 0;
        this.wantedAngle = wantedAngle;
        this.isAuto = DriverStation.isAutonomous();
        this.maxVel = isAuto ? PathsConstraints.AutoConstraints.MAX_VELOCITY : PathsConstraints.MAX_VELOCITY;
        this.accel = isAuto ? PathsConstraints.AutoConstraints.ACCEL : PathsConstraints.ACCEL;

        if (isRed)
            points = convertAlliance();
        fixFirstPoint(initialPose);

        initCorners();

        if (AvoidReef.isGoingThroughReef(new Segment(points.get(0).getTranslation(), points.get(1).getTranslation()))) {
            points = AvoidReef.fixPoints(points.get(0).getTranslation(), points.get(1).getTranslation(), wantedAngle);
        }

        createSegments();
        trajectoryLength = calcTrajectoryLength();
        distanceLeft = trajectoryLength;

        drivePID = new PIDController(2, 0, 0);
        this.isAlgae = isAlgae;
    }

    private void fixFirstPoint(Pose2d initialPose) {
        points.remove(0);
        points.add(0, new PathPoint(initialPose.getTranslation(), initialPose.getRotation()));

    }

    private ArrayList<PathPoint> convertAlliance() {
        ArrayList<PathPoint> convertedPoints = new ArrayList<PathPoint>();

        for (int i = 1; i < points.size(); i++) {
            convertedPoints.add(convertPoint(points.get(i)));
        }
        return convertedPoints;

    }

    private PathPoint convertPoint(PathPoint pointToConvert) {
        return new PathPoint(
                new Translation2d(FIELD_LENGTH - pointToConvert.getX(),
                        FIELD_HEIGHT - pointToConvert.getY()),
                pointToConvert.getRotation());
    }

    private void initCorners() {
        this.corners = new RoundedPoint[points.size() - 2];
        for (int i = 0; i < points.size() - 2; i++) {
            corners[i] = new RoundedPoint(points.get(i), points.get(i + 1), points.get(i + 2));
        }
    }

    private void createSegments() {

        if (points.size() < 3) {
            segments.add(new Leg(points.get(0).getTranslation(), points.get(1).getTranslation()));
        }

        else {
            segments.add(0, corners[0].getAtoCurveLeg());

            for (int i = 0; i < corners.length - 1; i++) {

                Arc arc = corners[i].getArc();
                if (arc.getPoints()[0].getDistance(arc.getPoints()[1]) >= 0.1)
                    segments.add(arc);
                segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart()));
            }

            Arc arc = corners[corners.length - 1].getArc();
            if (arc.getPoints()[0].getDistance(arc.getPoints()[1]) >= 0.1)
                segments.add(arc);
            segments.add(corners[corners.length - 1].getCtoCurveLeg());
        }
    }

    public double calcTrajectoryLength() {
        double sum = 0;
        for (Segment s : segments) {
            sum += s.getLength();
        }
        return sum;
    }

    public boolean hasFinishedSegments(Pose2d chassisPose) {
        Translation2d currentLastPoint = segmentIndex == segments.size() - 1
                ? segments.get(segmentIndex).getPoints()[1]
                : (segments.get(segmentIndex) instanceof Leg ? segments.get(segmentIndex).getPoints()[1]
                        : segments.get(segmentIndex + 1).getPoints()[0]);

        if (isAlgae)
            return chassisPose.getTranslation().getDistance(currentLastPoint) <= 0.1;

        if (segmentIndex == segments.size() - 1)
            return chassisPose.getTranslation().getDistance(currentLastPoint) <= MAX__POSITION_THRESHOLD;

        else {

            return chassisPose.getTranslation().getDistance(currentLastPoint) < 0.65;

        }
    }

    PIDController drivePID;


    

    private double getVelocity(double distanceFromLastPoint) {
        return Math.min(maxVel,
                Math.sqrt((distanceFromLastPoint * 2) / accel) * accel);
    }

    double lastDistance = 0;

    public ChassisSpeeds calculate(Pose2d chassisPose) {
        this.chassisPose = chassisPose;

        distanceTraveledOnSegment = segments.get(segmentIndex).distancePassed(chassisPose.getTranslation());
        distanceLeft -= (distanceTraveledOnSegment - lastDistance);
        lastDistance = distanceTraveledOnSegment;
        if (hasFinishedSegments(chassisPose)) {
            lastDistance = 0;
            if (segmentIndex != segments.size() - 1)
                segmentIndex++;
        }
        double velocity = getVelocity(chassisPose.getTranslation().getDistance(segments.get(segments.size() - 1).getPoints()[1]));

        Translation2d wantedVelocity = segments.get(segmentIndex).calcVector(chassisPose.getTranslation(), velocity);
        double wantedOmega = Math
                .abs(wantedAngle.minus(chassisPose.getRotation()).getRadians()) < MAX_ROTATION_THRESHOLD ? 0
                        : wantedAngle.minus(chassisPose.getRotation()).getRadians() * 0.9;

        if ((chassisPose.getTranslation()
                .getDistance(points.get(points.size() - 1).getTranslation()) <= MAX__POSITION_THRESHOLD
                && segmentIndex == segments.size() - 1))
            wantedVelocity = new Translation2d();

        return new ChassisSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), wantedOmega);
    }

    public boolean isFinishedTrajectory() {
        if (isAlgae)
            return chassisPose.getTranslation()
                    .getDistance(points.get(points.size() - 1).getTranslation()) <= 0.1;
        return ((chassisPose.getTranslation()
                .getDistance(points.get(points.size() - 1).getTranslation()) <= MAX__POSITION_THRESHOLD
                && segmentIndex == segments.size() - 1))

                && wantedAngle.minus(chassisPose.getRotation()).getRadians() <= MAX_ROTATION_THRESHOLD;
    }

}

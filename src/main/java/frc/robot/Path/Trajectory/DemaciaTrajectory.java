// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static frc.robot.Path.Trajectory.TrajectoryConstants.*;

import java.nio.file.attribute.AclEntry;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Path.Trajectory.TrajectoryConstants.PathsConstraints;
import frc.robot.Path.Utils.*;
import frc.robot.utils.LogManager;
import frc.robot.utils.Utils;

/** Add your docs here. */
public class DemaciaTrajectory {
    private ArrayList<Segment> segments;
    private double trajectoryLength;
    private double distanceTraveledOnSegment;
    private ArrayList<PathPoint> points;
    private ArrayList<PathPoint> pointsAfterFix;
    private RoundedPoint[] corners;
    private int segmentIndex;
    private Rotation2d wantedAngle;
    public double distanceLeft;
    Pose2d chassisPose = new Pose2d();
    
    
    double accel;

    double maxVel;

    /*
     * 
     * given points based on blue alliance
     * 
     */
    public DemaciaTrajectory(ArrayList<PathPoint> points, boolean isRed, Rotation2d wantedAngle, Pose2d initialPose) {
        this.segments = new ArrayList<Segment>();
        this.trajectoryLength = 0;
        this.distanceTraveledOnSegment = 0;
        this.points = points;
        this.segmentIndex = 0;
        this.wantedAngle = wantedAngle;
        this.maxVel = PathsConstraints.MAX_VELOCITY;

        if (isRed)
            points = convertAlliance();
        fixFirstPoint(initialPose);

        this.pointsAfterFix = new ArrayList<>();
        for(int i = 0; i < points.size() - 1; i++) {
            if (AvoidReef.isGoingThroughReef(new Segment(points.get(i).getTranslation(), points.get(i+1).getTranslation()))) {
                pointsAfterFix.addAll(AvoidReef.fixPoints(points.get(i).getTranslation(), points.get(i+1).getTranslation(), wantedAngle));
            } else{
                pointsAfterFix.add(points.get(i));
                if (i == points.size() - 2) {
                    pointsAfterFix.add(points.get(i+1));
                }
            }
        }

        this.points = pointsAfterFix;

      

        initCorners();


        createSegments();
        trajectoryLength = calcTrajectoryLength();
        distanceLeft = trajectoryLength;

        
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
                
                //segments.add(arc);
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

        if (segmentIndex == segments.size() - 1)
            return chassisPose.getTranslation().getDistance(currentLastPoint) <= MAX__POSITION_THRESHOLD;

        else {

            return chassisPose.getTranslation().getDistance(currentLastPoint) < 1;

        }
    }

    


    private double getVelocity(double distanceFromLastPoint, double currentVelocity) {
        
        if(distanceFromLastPoint < PathsConstraints.DISTANCE_TO_SLOWER_VELOCITY){
            return Math.max(PathsConstraints.FINISH_MAX_VELOCITY, PathsConstraints.FINISH_PID.calculate(distanceFromLastPoint, 0));
        }
        
        double v = Math.sqrt((distanceFromLastPoint * 2) / accel) * accel; 
        return Math.min(maxVel,
                Double.isNaN(v) ? 0 : v);
    }

    double lastDistance = 0;
    public ChassisSpeeds calculate(Pose2d chassisPose, double currentVelocity) {
        this.chassisPose = chassisPose;

        distanceTraveledOnSegment = segments.get(segmentIndex).distancePassed(chassisPose.getTranslation());
        distanceLeft -= (distanceTraveledOnSegment - lastDistance);
        lastDistance = distanceTraveledOnSegment;
        if (hasFinishedSegments(chassisPose)) {
            lastDistance = 0;
            if (segmentIndex != segments.size() - 1)
                segmentIndex++;
        }
        double velocity = getVelocity(chassisPose.getTranslation().getDistance(points.get(points.size() -1).getTranslation()), currentVelocity);
        
        Translation2d wantedVelocity = segments.get(segmentIndex).calcVector(chassisPose.getTranslation(), velocity);
        double diffAngle = wantedAngle.minus(chassisPose.getRotation()).getRadians();
        double wantedOmega = 0;
        
        if(Math.abs(diffAngle) > Math.toRadians(10)) wantedOmega = diffAngle * 2.2;
        else if(Math.abs(diffAngle) < MAX_ROTATION_THRESHOLD) wantedOmega = 0;
        else wantedOmega = diffAngle * 1.4;

        // SmartDashboard.putNumber("wanted velocity", wantedVelocity.getNorm());
        return new ChassisSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), wantedOmega);
    }


    public boolean isFinishedTrajectory() {
        
        return ((chassisPose.getTranslation()
                .getDistance(points.get(points.size() - 1).getTranslation()) <= MAX__POSITION_THRESHOLD
                && segmentIndex == segments.size() - 1))

                && wantedAngle.minus(chassisPose.getRotation()).getRadians() <= MAX_ROTATION_THRESHOLD;
    }

}

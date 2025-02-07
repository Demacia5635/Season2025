// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static frc.robot.Path.Trajectory.TrajectoryConstants.*;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Path.Utils.*;
import frc.robot.utils.LogManager;
/** Add your docs here. */
public class DemaciaTrajectory {
    private ArrayList<Segment> segments;
    private double trajectoryLength;
    private double distanceTraveled;
    private ArrayList<PathPoint> points;
    private RoundedPoint[] corners;
    private int segmentIndex;
    Rotation2d wantedAngle;
    Pose2d chassisPose = new Pose2d();
    TrapezoidProfile driveTrapezoid = new TrapezoidProfile(new Constraints(MAX_DRIVE_VELOCITY, MAX_DRIVE_ACCEL));
    
    /*
     * 
     * given points based on blue alliance
     * 
     */
    public DemaciaTrajectory(ArrayList<PathPoint> points, boolean isRed, Rotation2d wantedAngle, Pose2d initialPose) {
        this.segments = new ArrayList<Segment>();
        this.trajectoryLength = 0;
        this.distanceTraveled = 0;
        this.points = points;
        this.segmentIndex = 0;
        this.wantedAngle = wantedAngle;

        if (isRed) points = convertAlliance();
        fixFirstPoint(initialPose);

        initCorners();

        if (AvoidReef.isGoingThroughReef(new Segment(points.get(0).getTranslation(), points.get(1).getTranslation()))) {
            AvoidReef.fixPoints(points.get(0).getTranslation(), points.get(1).getTranslation(), wantedAngle);
        }

        createSegments();
        trajectoryLength = calcTrajectoryLength();

    }

    private void fixFirstPoint(Pose2d initialPose){
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
                segments.add(corners[i].getArc());
                segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart()));
            }
            segments.add(corners[corners.length - 1].getArc());
            segments.add(corners[corners.length - 1].getCtoCurveLeg());
        }
    }

    public double calcTrajectoryLength(){
        double sum = 0;
        for (Segment s : segments) {
            sum += s.getLength();
        }
        return sum;
    }

    public boolean hasFinishedSegments(Pose2d chassisPose){
        return segments.get(segmentIndex).distancePassed(chassisPose.getTranslation())
            >= segments.get(segmentIndex).getLength()- DISTANCE_OFFSET;
    }

    //PIDController omegaPidController = new PIDController(0.9, 0, 0);
    double kP = 0.9;
    public ChassisSpeeds calculate(Pose2d chassisPose, ChassisSpeeds currentVelocity) {
        this.chassisPose = chassisPose;
        
        if(hasFinishedSegments(chassisPose)) {
            distanceTraveled = segments.get(segmentIndex).distancePassed(chassisPose.getTranslation());
            if(segmentIndex != segments.size() - 1) segmentIndex++;
        }

        double lineDistance = points.get(points.size() - 1).getTranslation()
            .getDistance(chassisPose.getTranslation());

        Translation2d robotToTarget = points.get(points.size()- 1).getTranslation().minus(chassisPose.getTranslation()).times(-1);

        double vX = driveTrapezoid.calculate((lineDistance * robotToTarget.getAngle().getCos())/ MAX_DRIVE_VELOCITY, 
            new State(lineDistance * robotToTarget.getAngle().getCos(), currentVelocity.vxMetersPerSecond), 
            new State(0, 0)).velocity;


        double vY = driveTrapezoid.calculate((lineDistance * robotToTarget.getAngle().getSin())/ MAX_DRIVE_VELOCITY, 
            new State(lineDistance * robotToTarget.getAngle().getSin(), currentVelocity.vyMetersPerSecond), 
            new State(0, 0)).velocity;

        double velocity = Math.min(MAX_DRIVE_VELOCITY, Math.hypot(vX, vY));
        
        Translation2d wantedVelocity = segments.get(segmentIndex).calcVector(chassisPose.getTranslation(), velocity);

        double wantedOmega = Math.abs(wantedAngle.minus(chassisPose.getRotation()).getRadians()) < MAX_ROTATION_THRESHOLD ? 0
            : wantedAngle.minus(chassisPose.getRotation()).getRadians() * 0.9;
        

        return new ChassisSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), wantedOmega);
    }

    public boolean isFinishedTrajectory(){
        return (trajectoryLength - distanceTraveled <= MAX__POSITION_THRESHOLD
        || chassisPose.getTranslation().getDistance(points.get(points.size()-1).getTranslation()) <= MAX__POSITION_THRESHOLD) 
        
        && wantedAngle.minus(chassisPose.getRotation()).getRadians() <= MAX_ROTATION_THRESHOLD;
    }

}


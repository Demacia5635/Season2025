// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/** Add your docs here. */
public class DemaciaOdometry {
    private DemaciaKinematics kinematics;

    private Pose2d pose;
    private double SKIDDING_VALUE = 0.05;

    double lastAccelerometerValueX = 0;
    double lastAccelerometerValueY = 0;
    double STANDARD_SPIKE = 0.5;

    private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    private Rotation2d gyroOffset;
    private Rotation2d prevAngle;
    private SwerveModulePosition[] prevWheelPositions;

    public DemaciaOdometry(Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions,
            Pose2d initialPoseMeters, DemaciaKinematics kinematics) {
        this.kinematics = kinematics;
        pose = initialPoseMeters;
        gyroOffset = pose.getRotation().minus(gyroAngle);
        prevAngle = pose.getRotation();
        prevWheelPositions = kinematics.copy(wheelPositions);

    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        if(isColliding()) return pose; 
        var angle = gyroAngle.plus(gyroOffset);

        Twist2d twist = kinematics.toTwist2d(prevWheelPositions, wheelPositions);
        twist.dtheta = angle.minus(prevAngle).getRadians();

        
        if(isSkidding(wheelPositions)){
            twist.dx = twist.dx * SKIDDING_VALUE;
            twist.dy = twist.dy * SKIDDING_VALUE;
        }

        var newPose = pose.exp(twist);


        kinematics.copyInto(wheelPositions, prevWheelPositions);
        prevAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);

        return pose;
    }
    private boolean isSkidding(SwerveModulePosition[] wheelPosition){
        return false;
    }
    private boolean isColliding(){
        double diffX = accelerometer.getX() - lastAccelerometerValueX;
        double diffY = accelerometer.getY() - lastAccelerometerValueY;
        lastAccelerometerValueX = accelerometer.getX();
        lastAccelerometerValueY = accelerometer.getY();

        return ((accelerometer.getX() >= 2 || accelerometer.getY() >= 2) && (diffX > STANDARD_SPIKE || diffY > STANDARD_SPIKE));
        
    }
    public Pose2d getEstimatedPosition() {
        return pose;
    }
}

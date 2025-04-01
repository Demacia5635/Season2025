// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Paths;


import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Paths.PathsConstants.DRIVE_CONSTRAINTS;

/** Add your docs here. */
public class TrajectoryState extends Pose2d {
    public static TrajectoryState kEmptyState = new TrajectoryState(new Pose2d(), 0);

    private double velocityAtPoint;

    
    public TrajectoryState(Pose2d pose){
        this(pose, DRIVE_CONSTRAINTS.MAX_VELOCITY);
    }
    public TrajectoryState(Pose2d pose, double velocityAtPoint){
        super(pose.getTranslation(), pose.getRotation());
        this.velocityAtPoint = velocityAtPoint;
    }

    public double getVelocityAtPoint(){return this.velocityAtPoint;}
    public void setVelocityAtPoint(double velocity){this.velocityAtPoint = velocity;}

}

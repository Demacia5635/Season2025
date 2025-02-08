// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

/** Add your docs here. */
public class TrajectoryConstants {
    public static final double FIELD_LENGTH = -1; // X in field based
    public static final double FIELD_HEIGHT = -1; // y in field based
    
    
    public static final double MAX_ROTATIONAL_VELOCITY = Math.toRadians(360); //in radians;
    public static final double MAX_ROTATIONAL_ACCEL = Math.toRadians(720); // in radians^2
    public static final double MAX__POSITION_THRESHOLD = 0.05; // in meters;
    public static final double MAX_ROTATION_THRESHOLD = Math.toRadians(1.5);
    public static final double DISTANCE_OFFSET = 0.1; //in meters;

    public class PathsConstraints{
        public static final double MAX_APPROACH_VELOCITY = 2;
        public static final double MAX_APPROACH_ACCEL = 2.5;
        public static final double MAX_FINISH_VELOCITY = 1.5;
        public static final double MAX_FINISH_ACCEL = 3;
    }

}

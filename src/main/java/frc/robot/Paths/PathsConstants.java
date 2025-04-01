// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Paths;

/** Add your docs here. */
public class PathsConstants {
    public static class DRIVE_CONSTRAINTS {

        public static final double MAX_VELOCITY = 3;
        public static final double MAX_ACCEL = 6;
        public static final double MAX_JERK = 10;
    }
    public static class ROTATION_CONSTRAINTS{
        public static final double MAX_VELOCITY = Math.toRadians(360);
        public static final double MAX_ACCEL = Math.toRadians(720);
        public static final double MAX_JERK = Math.toRadians(1440);
    }

    public static final double MAX_POSITION_THRESHOLD = 0.03;
    public static final double MAX_TRAJECTORY_DISTANCE_THRESHOLD = 0.1;

}

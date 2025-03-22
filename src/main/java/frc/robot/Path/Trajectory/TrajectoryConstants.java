// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.utils.Utils;

/** Add your docs here. */
public class TrajectoryConstants {
    public static final double FIELD_LENGTH = -1; // X in field based
    public static final double FIELD_HEIGHT = -1; // y in field based

    public static final double MAX__POSITION_THRESHOLD = 0.03; // in meters;
    public static final double MAX_ROTATION_THRESHOLD = Math.toRadians(0.75);
    
   


    public class PathsConstraints {
        public static final double MAX_VELOCITY = 3.6;
        public static final double MAX_ACCEL = 8;
        public static final double FINISH_MAX_VELOCITY = 2;
        public static final double FINISH_ACCEL = 2;

        public static final ProfiledPIDController FINISH_PID = new ProfiledPIDController(0.9, 0, 0, new Constraints(FINISH_MAX_VELOCITY, FINISH_ACCEL));


        public static final double DISTANCE_TO_SLOWER_VELOCITY = 0.5; // Utils.distanceToDeaccel(MAX_VELOCITY, FINISH_MAX_VELOCITY, FINISH_ACCEL);
        public static final double MAX_ROTATIONAL_VELOCITY = Math.toRadians(360); // in radians;
        public static final double MAX_ROTATIONAL_ACCEL = Math.toRadians(720); // in radians^2

    }

}
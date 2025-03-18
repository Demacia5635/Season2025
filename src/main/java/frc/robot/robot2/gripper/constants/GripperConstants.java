// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.gripper.constants;

import edu.wpi.first.math.Pair;

/**
 * All gripper constants divided to diffrent static class based on all the
 * features of the gripper
 */
public class GripperConstants {

    /** the name of the subsystem */
    public static final String NAME = "Gripper";

    /** All the motor constants */
    public static class MotorConstants {
        public static final int MOTOR_ID = 30;
        public static final boolean INVERT = true;
        public static final boolean START_NEUTRAL_MODE = true;
    }

    /** All the sensor constants */
    public static class SensorConstants {
        // public static final int UP_FRONT_SENSOR_CHANNEL = 1;
        // public static final int UP_BACK_SENSOR_CHANNEL = 2;
        public static final Pair<Integer, Integer> DOWN_SENSOR_CHANNEL = new Pair<Integer,Integer>(1, 8);
        public static final Pair<Integer, Integer> UP_SENSOR_CHANNELS = new Pair<Integer, Integer>(3, 6);
        public static final double CORAL_IN_UP_SENSOR = 0.085;
        public static final double CORAL_IN_DOWN_SENSOR = 0.04;
    }

    /** All the constants for the grab command */
    public static class GrabConstants {
        public static final double FEED_POWER = 0.3;
    }
    
    public static class AlignCoralConstants {
        public static final double DOWN_POWER = 0.175;
        public static final double UP_POWER = -0.215;
    }

    /** All the constants for the drop command */
    public static class DropConstants {
        public static final double DROP_POWER = -0.5;
    }
}

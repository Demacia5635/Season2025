// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.constants;

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
        public static final boolean INVERT = false;
        public static final boolean START_NEUTRAL_MODE = true;
    }

    /** All the sensor constants */
    public static class SensorConstants {
        public static final int SENSOR_CHANNEL = 2;
        public static final double CORAL_IN_SENSOR = 3.9;
    }

    /** All the constants for the grab command */
    public static class GrabConstants {
        public static final double FEED_POWER = 0.35;
    }

    /** All the constants for the drop command */
    public static class DropConstants {
        public static final double DROP_POWER = 0.8;
        public static final double DROP_DURATION = 3;
    }

}

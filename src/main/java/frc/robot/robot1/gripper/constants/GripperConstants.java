// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.constants;

/** Add your docs here. */
public class GripperConstants {
    
    public static String NAME = "Gripepr";

    public static class MotorConstants {
        public static final int MOTOR_ID = 30;
        public static final boolean INVERT = false;
        public static final boolean BRAKE = true;
    }

    public static class SensorConstants {
        public static final int SENSOR_CHANNEL = 3;
        public static final double CORAL_IN_SENSOR = 3.9;
    }

    public static class GrabConstants {
        public static final double FEED_POWER = 0.35;
    }

    public static class DropConstants {
        public static final double DROP_POWER = 0.25;
        public static final double DROP_DURATION = 3;
    }

}

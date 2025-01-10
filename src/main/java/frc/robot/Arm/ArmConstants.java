// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class ArmConstants {
    public static class MOTORS_CONFIGS{
        //motor1 config
        public static final CANBus CANBUS = new CANBus("rio");
        public static final int MOTOR1_ID = 0;
        public static final String MOTOR1_NAME = "motor1";
        public static final double MOTOR1_KP = 0;
        public static final double MOTOR1_KI = 0;
        public static final double MOTOR1_KD = 0;
        public static final double MOTOR1_KS = 0;
        public static final double MOTOR1_KV = 0;
        public static final double MOTOR1_KA = 0;
        public static final double MOTOR1_KG = 0;
        public static final double MOTOR1_MAX_VELOCITY = 0;
        public static final double MOTOR1_MAX_Acceleration  = 0;
        public static final double MOTOR1_MAX_JERK = 0;
        public static final boolean IS_MOTOR1_BRAKE = true;
        public static final boolean IS_MOTOR1_INVERT = false;
        public static final double MOTOR1_GEAR_RATIO = 0;
        //motor2 config
        public static final int MOTOR2_ID = 0;
        public static final String MOTOR2_NAME = "motor2";
        public static final double MOTOR2_KP = 0;
        public static final double MOTOR2_KI = 0;
        public static final double MOTOR2_KD = 0;
        public static final double MOTOR2_KS = 0;
        public static final double MOTOR2_KV = 0;
        public static final double MOTOR2_KA = 0;
        public static final double MOTOR2_KG = 0;
        public static final double MOTOR2_MAX_VELOCITY = 0;
        public static final double MOTOR2_MAX_Acceleration  = 0;
        public static final double MOTOR2_MAX_JERK = 0;
        public static final boolean IS_MOTOR2_BRAKE = true;
        public static final boolean IS_MOTOR2_INVERT = false;
        public static final double MOTOR2_GEAR_RATIO = 0;
    }

    public static class ANGLES {
        public static final double DEFULT_ANGLE1 = 0;
        public static final double DEFULT_ANGLE2 = 0;
        public static final double L2_ANGLE1 = 0;
        public static final double L2_ANGLE2 = 0;
        public static final double L3_ANGLE1 = 0;
        public static final double L3_ANGLE2 = 0;
        public static final double TESTING_ANGLE1 = 0;
        public static final double TESTING_ANGLE2 = 0;
        public static final double PICK_ANGLE1 = 0;
        public static final double PICK_ANGLE2 = 0;
        public static final double ANGLE1_RANGE = 0;
        public static final double ANGLE2_RANGE = 0;
    }

    public enum STATE{
        DEFAULT, PICK, L2, L3, TESTING
    }
}

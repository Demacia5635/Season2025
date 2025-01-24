// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utils.TalonConfig;

/**
 * The arm constants
 * <br>
 * </br>
 * This class is divided to couple of diffrent static class for each part in the
 * arm
 */
public class ArmConstants {

    /** The name of the Subsystem */
    public static final String NAME = "Arm";

    /**
     * All the constnats of the calculation
     */
    public static class CalculationsConstants {
        public static final double BASE_HEIGHT = 0.846;
        public static final double ARM_1_LEN = 0.410;
        public static final double ARM_2_LEN = 0.255;
    }

    /**
     * All the constants of the arm angle motor
     */
    public static class ArmAngleMotorConstants {
        /* all the main configs of the motor */
        public static final int ID = 20;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = Constants.CAN_BUS;
        public static final String NAME = "Arm Angle Motor";

        /* the pid and ff constants of the motor */
        public static final double KP = 20;
        public static final double KI = 0.75;
        public static final double KD = 0.5;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 1.5;
        public static final double MOTION_MAGIC_ACCELERATION = 3;
        public static final double MOTION_MAGIC_JERK = 6;

        /* the channel of the limit switch of the arm angle motor */
        public static final int LIMIT_SWITCH_CHANNEL = 2;

        /* the basic configues of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (105.0 / 16.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double BASE_ANGLE = Math.toRadians(33.7);
        public static final double BACK_LIMIT = Math.toRadians(33.7);
        public static final double FWD_LIMIT = Math.toRadians(130);

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);
    }

    /**
     * the constants for the gripper going automaticly to a specific angle when the
     * arm is less than a specific angle
     */
    public static class GripperAngleStarting {
        public static final double WHEN_MOVING_GRIPPER = Math.toRadians(38);
        public static final double ANGLE_TO_GRIPPER = 3.64;
    }

    /** All the constants for the gripper angle motor */
    public static class GripperAngleMotorConstants {
        /* All the main configs of the motor */
        public static final int ID = 21;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = Constants.CAN_BUS;
        public static final String NAME = "Gripper Angle Motor";

        /* the pid and ff of the motor */
        public static final double KP = 8;
        public static final double KI = 1.5;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        /* the channel of the absolute sensor */
        public static final int ABSOLUTE_SENSOR_CHANNEL = 1;

        /* all the basic configs of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 36.0 * (25.0 / 18.0) * (49.0 / 83.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.8;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double ENCODER_BASE_ANGLE = -1.6270964174482998;
        public static final double BACK_LIMIT = 3.6362957660963287;
        public static final double FWD_LIMIT = 5.3824149977630915;

        /* The config of the motor based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);
    }

    /** All the constants of the calibration */
    public static class CalibrationConstants {
        public static final double ARM_ANGLE_POWER = -0.2;
        public static final double ARM_ANGLE_START_POWER = 0.2;
        public static final double TIME_TO_CHANGE_POWER = 0.5;
    }

    /** The max errors of the arm and gripper angles */
    public static class MaxErrors {
        public static final double ARM_ANGLE_ERROR = 0.05;
        public static final double GRIPPER_ANGLE_ERROR = 0.05;
    }

    /** all the constants angles */
    public static class ANGLES {
        public static final Pair<Double, Double> L2 = new Pair<Double, Double>(0.872665, 5d);
        public static final Pair<Double, Double> L3 = new Pair<Double, Double>(1.570796, 4d);
        public static final Pair<Double, Double> CORAL_STATION = new Pair<Double, Double>(1.919862, 5.3824149977630915);
        public static final Pair<Double, Double> ALGAE_UNDER = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> ALGAE_OVER = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> STARTING = new Pair<Double, Double>(Math.toRadians(33.7), 3.64);
    }

    /** the arm angle states */
    public static enum ARM_ANGLE_STATES {
        L2_CALC,
        L3_CALC,
        L2_TOUCHING,
        L3_TOUCHING,
        ALGAE_UNDER,
        ALGAE_OVER,
        CORAL_STATION,
        TESTING,
        STARTING,
        IDLE;

        ARM_ANGLE_STATES() {
        }
    }

    /**
     * the field objects used for calculating what angles the arm needs to be even
     * if its not perfectly in the field objects
     * TODO: needs to remove this class after class of the field is created
     */
    public static class FieldConstants {
        public static final Translation2d REEF = new Translation2d();
        public static final Translation2d CORAL_STATION = new Translation2d();

        public static final double L2_HEIGHT = 0;
        public static final double L3_HEIGHT = 0;
        public static final double CORAL_STATION_HEIGHT = 0;
    }
}

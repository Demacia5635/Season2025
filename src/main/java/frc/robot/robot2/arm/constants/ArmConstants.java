// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.arm.constants;

import org.opencv.core.Mat;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CANBuses;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
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
    // public static class CalculationsConstants {//TODO: what is that?
    //     public static final double BASE_HEIGHT = 0.86;
    //     public static final double ARM_1_LEN = 0.53;
    //     public static final double ARM_2_LEN = 0.32;
    // }

    /**
     * All the constants of the arm angle motor
     */
    public static class ArmAngleMotorConstants {

        public static final double Angle_OFFSET = Math.toRadians(0.0);//TODO: find right offset

        /* all the main configs of the motor */
        public static final int ID = 20;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Arm Angle Motor";

        /* All the main configs of the cancoder*/
        public static final int CANCODER_ID = 22;//TODO: find right id
        public static final String CANCODER_NAME = "Arm Cancoder";


        /* the pid and ff constants of the motor */ //TODO: find kp ki kd and maybe ks kv ka m kg
        public static final double KP = 20;
        public static final double KI = 1.0;
        public static final double KD = 0.5;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        /* the motion magic constants of the motor */  //TODO: CHECK IF IT IS WORKING
        public static final double MOTION_MAGIC_VELOCITY = 1.5;
        public static final double MOTION_MAGIC_ACCELERATION = 3;
        public static final double MOTION_MAGIC_JERK = 6;

        /* the channel of the absolute sensor */
        public static final int ABSOLUTE_SENSOR_CHANNEL = 1;

        /* the channel of the limit switch of the arm angle motor */
        public static final int LIMIT_SWITCH_CHANNEL = 0;

        /* the basic configues of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 48.0 * (60.0 / 48.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double BASE_ANGLE = 0; //TODO:
        public static final double BACK_LIMIT = Math.toRadians(33.7);
        public static final double FWD_LIMIT = 2.501220703125;

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);

        /* The config of the motor based on the constants cancoder */
        public static final CancoderConfig CANCODER_CONFIG = new CancoderConfig(CANCODER_ID,CAN_BUS,CANCODER_NAME);
    }

    /**
     * The constants for the gripper going automaticly to a specific angle when the
     * arm is less than a specific angle
     */
    public static class GripperAngleStarting {
        public static final double WHEN_MOVING_GRIPPER = Math.toRadians(38);
        public static final double ANGLE_TO_GRIPPER = 3.75;
    }

    /** All the constants for the gripper angle motor */
    public static class GripperAngleMotorConstants {

        public static final double Angle_OFFSET = Math.toRadians(0.0);//TODO: find right offset

        /* All the main configs of the cancoder*/
        public static final int CANCODER_ID = 22;//TODO: find right id
        public static final String CANCODER_NAME = "Gripper Cancoder";


        /* All the main configs of the motor */
        public static final int ID = 21;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Gripper Angle Motor";

        /* the pid and ff of the motor */ //TODO: find kp ki kd and maybe ks kv ka m kg
        public static final double KP = 9.5;
        public static final double KI = 1.75;
        public static final double KD = 0.25;
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
        public static final double GEAR_RATIO = 48.0 * (57.0 / 48.0);

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double ENCODER_BASE_ANGLE = -1.6270964174482998;
        public static final double BACK_LIMIT = 3.7;
        public static final double FWD_LIMIT = 5.4;

        /* The config of the motor based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);
 
        /* The config of the motor based on the constants cancoder */

    }

    /** All the constants of the calibration */
    public static class CalibrationConstants {
        public static final double ARM_ANGLE_POWER = -0.2;
        public static final double ARM_ANGLE_START_POWER = 0.2;
        public static final double TIME_TO_CHANGE_POWER = 0.75;
    }

    /** The max errors of the arm and gripper angles */
    public static class MaxErrors {
        public static final double ARM_ANGLE_UP_ERROR = 0.017;
        public static final double ARM_ANGLE_DOWN_ERROR = 0.03;
        public static final double GRIPPER_ANGLE_UP_ERROR = 0.017;
        public static final double GRIPPER_ANGLE_DOWN_ERROR = 0.03;
    }

    /** all the constants angles */
    public static class ANGLES {
        public static final Pair<Double, Double> L2 = new Pair<Double, Double>(1.8, 4.4);
        public static final Pair<Double, Double> L3 = new Pair<Double, Double>(2.4, 4.4);
        public static final Pair<Double, Double> L4 = new Pair<Double,Double>(0.0, 0.0);//TODO: find angles
        public static final Pair<Double, Double> CORAL_STATION = new Pair<Double, Double>(1.54, 5.3);
        public static final Pair<Double, Double> ALGAE_BOTTOM = new Pair<Double, Double>(1.55, 4.4);
        public static final Pair<Double, Double> ALGAE_TOP = new Pair<Double, Double>(2.2, 4.4);
        public static final Pair<Double, Double> STARTING = new Pair<Double, Double>(Math.toRadians(33.7), 3.64);
    }

    /** the arm angle states */
    public static enum ARM_ANGLE_STATES {
        L2,
        L3,
        L4,
        ALGAE_BOTTOM,
        ALGAE_TOP,
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

        public static final double L2_HEIGHT = 0.81;
        public static final double L3_HEIGHT = 1.02;
        public static final double L4_HEIGHT = 1.82866;//TODO:find hight
        public static final double CORAL_STATION_HEIGHT = 0.95098;
    }
}

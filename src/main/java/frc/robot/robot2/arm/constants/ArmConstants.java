// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.arm.constants;

import org.opencv.core.Mat;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.CANBuses;
import frc.robot.robot2.arm.subsystems.Arm;
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

        public static final double Angle_OFFSET = -2.9728577121654854436248148071636; 

        /* all the main configs of the motor */
        public static final int ID = 20;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Arm Angle Motor";

        /* All the main configs of the cancoder*/
        public static final int CANCODER_ID = 22;//TODO: find right id
        public static final String CANCODER_NAME = "Arm Cancoder";


        /* the pid and ff constants of the motor */ //TODO: find kp ki kd and maybe ks kv ka m kg 
        //Top

        public static final double[] KP = {3.5, 5, 5};
        public static final double[] KI = {0, 0, 0};
        public static final double[] KD = {0, 0, 0};
        public static final double[] KS = {0, 0.2, 0.2};
        public static final double[] KV = {1.5, 1.5, 1.5};
        public static final double[] KA = {0.003, 0.003, 0.003};
        public static final double[] KG = {0, 0.1375, -0.1375};

        /* the motion magic constants of the motor */  //TODO: CHECK IF IT IS WORKING
        public static final double MOTION_MAGIC_VELOCITY = 4;
        public static final double MOTION_MAGIC_ACCELERATION = 6;
        public static final double MOTION_MAGIC_JERK = 8;

        /* the channel of the absolute sensor */
        //public static final int ABSOLUTE_SENSOR_CHANNEL = 22;

        /* the basic configues of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = true;
        public static final double GEAR_RATIO = 80;

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 1.1;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        //public static final double BASE_ANGLE = 0; //TODO:
        //public static final double BACK_LIMIT = -2.569414874465336;
        public static final double BACK_LIMIT = -2.5;
        
        public static final double FWD_LIMIT = 2.569414874465336;

        /* The config of the motors based on the constants above */
        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP[0], KI[0], KD[0], KS[0], KV[0], KA[0], 0)
                .withPID1(KP[1], KI[1], KD[1], KS[1], KV[1], KA[1], 0)
                .withPID2(KP[2], KI[2], KD[2], KS[2], KV[2], KA[2], 0)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME);

        /* The config of the motor based on the constants cancoder */
        public static final CancoderConfig CANCODER_CONFIG = new CancoderConfig(CANCODER_ID,CAN_BUS,CANCODER_NAME)
            .withInvert(IS_INVERTED);
    }

    public static class ArmFFLookupTable {
        public static final double[][] values = {
            {-2.204327446948554, 0.275, -0.15},
            {-1.1642884727620881, 0.275, -0.2},
            {-0.05, 0.22, -0.23},
            {1.1642884727620881, 0.2, -0.275},
            {2.204327446948554, 0.15, -0.275},
        };
    }

    /** All the constants for the gripper angle motor */
    public static class GripperAngleMotorConstants {



        /* All the main configs of the motor */
        public static final int ID = 21;
        public static final CANBus CAN_BUS = CANBuses.ARM_CAN_BUS;
        public static final String NAME = "Gripper Angle Motor";

        /* the pid and ff of the motor */ //TODO: find kp ki kd and maybe ks kv ka m kg
        public static final double KP = 4;
        public static final double KI = 0.5;
        public static final double KD = 0.1;
        public static final double KS = 0.1;
        public static final double KV = 1.0065855602048472;
        public static final double KA = 0.0025034812871556067;
        public static final double KG = 0;

        /* the motion magic constants of the motor */
        public static final double MOTION_MAGIC_VELOCITY = 2;
        public static final double MOTION_MAGIC_ACCELERATION = 6;
        public static final double MOTION_MAGIC_JERK = 10;

        /* the channel of the absolute sensor */
        public static final int ABSOLUTE_SENSOR_CHANNEL = 2;

        /* all the basic configs of the motor */
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 52;

        /* the ramp time of the motor */
        public static final double RAMP_TIME = 0.5;

        /*
         * all the angles of the motor
         * base -> where the limit switch
         * back limit -> the minimum angle
         * forward limit -> the maximum angle
         */
        public static final double Angle_OFFSET = 2.6171682281515816 + Math.PI/2;
        public static final double BACK_LIMIT = -2.81;
        public static final double FWD_LIMIT = -0.2438946850555146;

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

    /** The max errors of the arm and gripper angles */
    public static class MaxErrors {
        public static final double ARM_ANGLE_UP_ERROR = 0.017;
        public static final double ARM_ANGLE_DOWN_ERROR = 0.03;
        public static final double GRIPPER_ANGLE_UP_ERROR = 0.017;
        public static final double GRIPPER_ANGLE_DOWN_ERROR = 0.03;
    }
}

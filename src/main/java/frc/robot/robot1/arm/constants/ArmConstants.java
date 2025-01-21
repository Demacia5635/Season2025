// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ArmConstants {
    
    public static final String NAME = "Arm";

    public static class CalculationsConstants {
        public static final double BASE_HEIGHT = 0.846;
        public static final double ARM_1_LEN = 0.410;
        public static final double ARM_2_LEN = 0.255;
    }

    public static class ArmAngleMotorConstants {
        public static final int ID = 20;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = Constants.CAN_BUS;
        public static final String NAME = "Arm Angle Motor";

        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        public static final int LIMIT_SWITCH_CHANNEL = 0;

        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = true;
        public static final double GEAR_RATIO = 1.0 / 36.0 * (16.0 / 105.0);

        /* The angle where the limit switch */
        public static final double BASE_ANGLE = 0;
        public static final double BACK_LIMIT = 0;
        public static final double FWD_LIMIT = 0;

        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO);
    }

    public static class GripperAngleMotorConstants {
        public static final int ID = 21;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = Constants.CAN_BUS;
        public static final String NAME = "Gripper Angle Motor";

        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;

        public static final int LIMIT_SWITCH_CHANNEL = 0;

        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = true;
        public static final double GEAR_RATIO = 1.0 / 36.0 * (18.0 / 25.0) * (83.0 / 49.0);

        /* The angle in the limit switch */
        public static final double BASE_ANGLE = 0;
        public static final double BACK_LIMIT = 0;
        public static final double FWD_LIMIT = 0;

        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor();
    }

    public static class TeethRatios {
        public static final double ARM_ANGLE_MINIMUM_VELOCITY = 0.2;
        public static final double GRIPPER_ANGLE_MINIMUM_VOLTAGE = 1;
        public static final double GEARS_RATIO = (16.0 / 105.0) / (18.0 / 25.0);
    }

    public static class CalibrationConstants {
        public static final double ARM_ANGLE_POWER = -0.1;
        public static final double GRIPPER_ANGLE_POWER = -0.1;
    }

    public static class MaxErrors {
        public static final double ARM_ANGLE_ERROR = 0.25;
        public static final double GRIPPER_ANGLE_ERROR = 0.25;
    }
    
    public static class ANGLES {
        public static final Pair<Double, Double> L2 = new Pair<Double,Double>(0.0, 0.0);
        public static final Pair<Double, Double> L3 = new Pair<Double,Double>(0.0, 0.0);
        public static final Pair<Double, Double> CORAL_STATION = new Pair<Double,Double>(0.0, 0.0);
        public static final Pair<Double, Double> ALGAE_UNDER = new Pair<Double,Double>(0.0, 0.0);
        public static final Pair<Double, Double> ALGAE_OVER = new Pair<Double,Double>(0.0, 0.0);
    }
    
    public static enum ARM_ANGLE_STATES {
        L2_CALC,
        L3_CALC,
        L2_TOUCHING,
        L3_TOUCHING,
        ALGAE_UNDER,
        ALGAE_OVER,
        CORAL_STATION,
        TESTING,
        IDLE;

        ARM_ANGLE_STATES() {
        }
    }

    /* TODO: remove when creating field utils for everyone */
    public static class FieldConstants {
        public static final Translation2d REEF = new Translation2d();
        public static final Translation2d CORAL_STATION = new Translation2d();
        
        public static final double L2_HEIGHT = 0;
        public static final double L3_HEIGHT = 0;
        public static final double CORAL_STATION_HEIGHT = 0;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ArmConstants {

    public static class CalculationsConstants {
        public static final double BASE_HEIGHT = 0.845;
        public static final double ARM_1_LEN = 0.535;
        public static final double ARM_2_LEN = 0.1;
    }

    public static final String NAME = "Arm";

    public static class ArmAngleMotorConstants {
        public static final int ID = 0;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = new CANBus();
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
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 0;

        /* The angle where the limit switch */
        public static final double BASE_ANGLE = 0;

        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor();
    }

    public static class GripperAngleMotorConstants {
        public static final int ID = 0;
        /* TODO: change to canbus at constants when merging the branch */
        public static final CANBus CAN_BUS = new CANBus();
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
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 0;

        /* The angle in the limit switch */
        public static final double BASE_ANGLE = 0;

        public static final TalonConfig CONFIG = new TalonConfig(ID, CAN_BUS, NAME)
                .withPID(KP, KI, KD, KS, KV, KA, KG)
                .withMotionMagic(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
                .withBrake(IS_BRAKE)
                .withInvert(IS_INVERTED)
                .withMotorRatio(GEAR_RATIO).withRadiansMotor();
    }

    public static class CalibrationConstants {
        public static final double ARM_ANGLE_POWER = -0.1;
        public static final double GRIPPER_ANGLE_POWER = -0.1;
    }

    public static enum ARM_ANGLE_STATES {
        L2(0),
        L3(0),
        CORAL_STATION(0),
        TESTING(0),
        IDLE(0);

        public final double TARGET_HEIGHT;

        ARM_ANGLE_STATES(double targetHeight) {
            this.TARGET_HEIGHT = targetHeight;
        }
    }

    /* TODO: remove when creating field utils for everyone */
    public static class FieldConstants {
        public static final Translation2d REEF = new Translation2d();
        public static final Translation2d CORAL_STATION = new Translation2d();
    }
}

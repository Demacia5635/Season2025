// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator;

import com.ctre.phoenix6.CANBus;

import frc.robot.Constants.CANBuses;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ElevatorConstants {

    public static final String NAME = "Elevator";

    public static class ElevatorMotorConstants {
        public static final double DELATA_TIME = 0.02;
        private static final int MOTOR_ID = 60;
        private static final CANBus CANBUS = CANBuses.ARM_CAN_BUS;
        private static final String NAME = "Elevator/Motor";
        private static final double GEAR_RATIO = 1.0/0.007;
        public static final double kP = 40;
        public static final double kI = 20;
        public static final double kD = 15;
        public static final double kG = 0.22614986964940442;
        public static final double kV = 16;
        public static final double kS = 0;
        public static final double kA = 0;
        private static final double VELOCITY = 0;
        private static final double ACCELERATION = 0;
        private static final double JERK = 0;
        private static final boolean NETURAL_MODE = true;
        private static final boolean IS_INVERTED = true;
        private static final double RAMP_TIME = 0.15;
        public static final double MAX_VEL = 0.4;
        public static final double ACC = 2;
        public static final double DEACCEL_DISTANCE = 0.5 * MAX_VEL * MAX_VEL / ACC;
        public static final double DELATA_VEL = MAX_VEL * DELATA_TIME;
        public static final double ALLOWED_ERROR = 0.02;
        /*
         * KS = 0.06474606582845017
KV = 16.634375086155174 / 0.38188467058072995
KA = 0.04204649456490994
KG= 0.22614986964940442
         */
        
        public static final TalonConfig motorConfig = 
        new TalonConfig(MOTOR_ID, CANBUS, NAME).withMotorRatio(GEAR_RATIO)
        .withPID(kP, kI, kD, 0, kV, 0, kG)
        .withMotionMagic(VELOCITY, ACCELERATION, JERK)
        .withBrake(NETURAL_MODE).withRampTime(RAMP_TIME).withInvert(IS_INVERTED);
    }

    public class ElevatorLimits{
        public static final double TOP_LIMIT_POSITION = 0.6;
        public static final double BOTTOM_LIMIT_POSITION = 0.01;
        public static final int TOP_SWITCH_ID = 0;
        public static final int BOTTOM_SWITCH_ID = 1;
    }

    public static class CalibrationConstants {
        public static final double POWER = 0.2;
        public static final double HEIGHT = 0;
    }

    public enum ELEVATOR_STATE {
        L4(0.55),
        L3(0.5),
        L2(0.2),
        CORAL_STATION(0.1),
        STARTING(0),
        TESTING(0.3),
        IDLE(0);

        public final double HEIGHT;

        private ELEVATOR_STATE(double height) {
            this.HEIGHT = height;
        }
    }
}

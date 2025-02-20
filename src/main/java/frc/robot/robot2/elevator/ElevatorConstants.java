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

        private static final int MOTOR_ID = 60;
        private static final CANBus CANBUS = CANBuses.ARM_CAN_BUS;
        private static final String NAME = "Elevator/Motor";
        private static final double GEAR_RATIO = 1.0/0.007;
        private static final double kP = 40;
        private static final double kI = 20;
        private static final double kD = 15;
        private static final double kG = 0.22614986964940442;
        private static final double kV = 16;
        private static final boolean NETURAL_MODE = true;
        private static final boolean IS_INVERTED = true;
        private static final double RAMP_TIME = 0.3;
        /*
         * KS = 0.06474606582845017
KV = 16.634375086155174 / 0.38188467058072995
KA = 0.04204649456490994
KG= 0.22614986964940442
         */
        
        public static final TalonConfig motorConfig = 
        new TalonConfig(MOTOR_ID, CANBUS, NAME).withMotorRatio(GEAR_RATIO)
        .withPID(kP, kI, kD, 0, kV, 0, kG)
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

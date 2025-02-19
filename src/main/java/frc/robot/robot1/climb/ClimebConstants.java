// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.climb;

import frc.robot.Constants;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ClimebConstants {

    /** the name of the subsystem */
    public static final String NAME = "Climb";

    public static final TalonConfig MOTOR_CONFIG = 
            new TalonConfig(MotorConstants.MOTOR_ID, Constants.CANBuses.ARM_CAN_BUS, NAME + "Motor")
            .withInvert(MotorConstants.INVERT)
            .withBrake(MotorConstants.START_NEUTRAL_MODE)
            .withMotorRatio(ClimbConstants.CLIMB_RATIO)
            .withRadiansMotor();
    /** All the motor constants */
    public static class MotorConstants {
        public static final int MOTOR_ID = 40;
        public static final boolean INVERT = false;
        public static final boolean START_NEUTRAL_MODE = false;
    }
    public static class ClimbConstants {
        public static final double prepareClimbPower = 0.8;
        public static final double prepareClimbTime = 2;
        public static final double STALL_CURRENT = 35;
        public static final double CLIMB_POWER = -0.3;
        public static final double CLIMB_RATIO = 600;
        public static final double HAS_CLIMED_ANGLE = -1.4;
    }
    public static final int LIMIT_SWITCH_CHANNEL = 4;
    
}

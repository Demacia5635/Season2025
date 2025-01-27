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

    private static int MOTOR_ID = -1;
    private static CANBus CANBUS = CANBuses.CHASSIS_CAN_BUS;
    private static double GEAR_RATIO = -1/-1;
    private static final double kP = -1;
    private static final double kI = -1;
    private static final double kD = -1;
    private static final double kG = -1;
    private static final double kV = -1;
    private static final boolean NETURAL_MODE = true;
    private static final boolean IS_INVERTED = false;
    private static final double CIRCUMFERENCE = -1;
    private static final double RAMP_TIME = 0.3;

    public static final TalonConfig motorConfig = 
    new TalonConfig(MOTOR_ID, CANBUS, "Elevator Motor").withMotorRatio(GEAR_RATIO)
    .withPID(kP, kI, kD, 0, kV, 0, kG)
    .withBrake(NETURAL_MODE).withRampTime(RAMP_TIME).withInvert(IS_INVERTED)
    .withMeterMotor(CIRCUMFERENCE);
}

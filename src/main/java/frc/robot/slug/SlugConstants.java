// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.slug;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class SlugConstants {
    public static final int MOTOR_ID = 0;
    public static final CANBus CANBUS = new CANBus("rio");
    public static final String NAME = "Gripper";
    public static final TalonConfig MOTOR_CONFIG = new TalonConfig(MOTOR_ID, CANBUS, NAME);

    public static final int SENSOR_CHANNEL = 0;

    public static final double FEED_POWER = 0;

    public static final double EXCRETE_POWER = 0;
    public static final double EXCRETE_DURATION = 3;
}

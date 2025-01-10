package frc.robot.chassis;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

public class ChassisConstants {
    public static final double MAX_DRIVE_VELOCITY = 3.6;
    public static final double MAX_OMEGA_VELOCITY = Math.toRadians(360);
    public static final int GYRO_ID = 14;
    public static final CANBus CANBus = new CANBus("rio");
    public static final double WHEEL_DIAMETER = 0.1016; // 4 inch
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double STEER_GEAR_RATIO = 151.0/7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    
    public static class SwerveModuleConfigs {
        public final TalonConfig STEER_CONFIG;
        public final TalonConfig DRIVE_CONFIG;
        public final CancoderConfig CANCODER_CONFIG;
        public final Translation2d POSITION;
        public final double STEER_OFFSET;

        public SwerveModuleConfigs(TalonConfig steerConfig, TalonConfig driveConfig, CancoderConfig cancoderConfig, Translation2d position, double steerOffset) {
            STEER_CONFIG = steerConfig;
            DRIVE_CONFIG = driveConfig;
            CANCODER_CONFIG = cancoderConfig;
            POSITION = position;
            STEER_OFFSET = steerOffset;
        }
    }
    
    public static final double STEER_KP = 2;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.06617640545050964;
    public static final double STEER_KV = 0.3866402641515461;
    public static final double STEER_KA = 0.05;

    public static final double DRIVE_KP = 5;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0.004;
    public static final double DRIVE_KS = 0.2;
    public static final double DRIVE_KV = 0.45;
    public static final double DRIVE_KA = 0.004;

    public static final double MOTION_MAGIC_VEL = 15 * 2 * Math.PI;
    public static final double MOTION_MAGIC_ACCEL = 8 * 2 * Math.PI;
    public static final double MOTION_MAGIC_JERK = 160 * 2 * Math.PI;

    public static final double RAMP_TIME_STEER = 0.25;

    public static final SwerveModuleConfigs FRONT_LEFT = new SwerveModuleConfigs(
        new TalonConfig(2, CANBus, "Front Left Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
            .withMotionMagic(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
            .withBrake(true).withMotorRatio(STEER_GEAR_RATIO).withRadiansMotor()
            .withRampTime(RAMP_TIME_STEER),
        new TalonConfig(1, CANBus, "Front Left Drive")
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
            .withBrake(true)
            .withInvert(true)
            .withMotorRatio(DRIVE_GEAR_RATIO).withMeterMotor(WHEEL_CIRCUMFERENCE),
        new CancoderConfig(3, CANBus, "Front Left Cancoder"),
        new Translation2d(0.34, 0.29),
        -0.54380968833639320957788356964568
    );
    public static final SwerveModuleConfigs FRONT_RIGHT = new SwerveModuleConfigs(
        new TalonConfig(5, CANBus, "Front Right Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
            .withMotionMagic(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
            .withBrake(true).withMotorRatio(STEER_GEAR_RATIO).withRadiansMotor()
            .withRampTime(RAMP_TIME_STEER),
        new TalonConfig(4, CANBus, "Front Right Drive")
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
            .withBrake(true)
            .withInvert(true)
            .withMotorRatio(DRIVE_GEAR_RATIO).withMeterMotor(WHEEL_CIRCUMFERENCE),
        new CancoderConfig(6, CANBus, "Front Right Cancoder"),
        new Translation2d(0.34, -0.29),
        0.05906194188748811288309769560565
    );
    public static final SwerveModuleConfigs BACK_LEFT = new SwerveModuleConfigs(
        new TalonConfig(8, CANBus, "Back Left Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
            .withMotionMagic(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
            .withBrake(true).withMotorRatio(STEER_GEAR_RATIO).withRadiansMotor()
            .withRampTime(RAMP_TIME_STEER),
        new TalonConfig(7, CANBus, "Back Left Drive")
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
            .withBrake(true)
            .withInvert(true)
            .withMotorRatio(DRIVE_GEAR_RATIO).withMeterMotor(WHEEL_CIRCUMFERENCE),
        new CancoderConfig(9, CANBus, "Back Left Cancoder"),
        new Translation2d(-0.34, 0.29),
        2.7909909134491723130502123817055
    );
    public static final SwerveModuleConfigs BACK_RIGHT = new SwerveModuleConfigs(
        new TalonConfig(11, CANBus, "Back Right Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
            .withMotionMagic(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
            .withBrake(true).withMotorRatio(STEER_GEAR_RATIO).withRadiansMotor()
            .withRampTime(RAMP_TIME_STEER),
        new TalonConfig(10, CANBus, "Back Right Drive")
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
            .withBrake(true)
            .withInvert(true)
            .withMotorRatio(DRIVE_GEAR_RATIO).withMeterMotor(WHEEL_CIRCUMFERENCE),
        new CancoderConfig(12, CANBus, "Back Right Cancoder"),
        new Translation2d(-0.34, -0.29),
        -0.70563312592280345929109433031841
    );
}
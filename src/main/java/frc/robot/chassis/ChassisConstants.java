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
    
    public static final double STEER_KP = 4.1;
    public static final double STEER_KI = 0.9;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.19817640545050964;
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
    
    public static class SwerveModuleConfigs {
        public final TalonConfig STEER_CONFIG;
        public final TalonConfig DRIVE_CONFIG;
        public final CancoderConfig CANCODER_CONFIG;
        public final Translation2d POSITION;
        public final double STEER_OFFSET;
        public final String NAME;

        public SwerveModuleConfigs(TalonConfig steerConfig, TalonConfig driveConfig, CancoderConfig cancoderConfig, Translation2d position, double steerOffset, String name) {
            STEER_CONFIG = steerConfig;
            DRIVE_CONFIG = driveConfig;
            CANCODER_CONFIG = cancoderConfig;
            POSITION = position;
            STEER_OFFSET = steerOffset;
            NAME = name;
        }
        
        public SwerveModuleConfigs(int swerveId, double steerOffset) {
            switch (swerveId) {
                case 0:
                    NAME = "Front Left";
                    break;
                case 1:
                    NAME = "Front Right";
                    break;
                case 2:
                    NAME = "Back Left";
                    break;
                case 3:
                    NAME = "Back Right";
                    break;
            
                default:
                    NAME = "";
                    break;
            }
            STEER_CONFIG = new TalonConfig(swerveId * 3 + 2, CANBus, NAME + " Steer")
                .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
                .withMotionMagic(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
                .withBrake(true)
                .withMotorRatio(STEER_GEAR_RATIO).withRadiansMotor()
                .withRampTime(RAMP_TIME_STEER);
            DRIVE_CONFIG = new TalonConfig(swerveId * 3 + 1, CANBus, NAME + " Drive")
                .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, STEER_KS, STEER_KV, STEER_KA, 0)
                .withBrake(true)
                .withInvert(true)
                .withMotorRatio(DRIVE_GEAR_RATIO).withMeterMotor(WHEEL_CIRCUMFERENCE);
            CANCODER_CONFIG = new CancoderConfig(swerveId * 3 + 3, CANBus, NAME + " Cancoder");
            POSITION = new Translation2d(
                swerveId == 0 || swerveId == 1 ? 0.34 : -0.34,
                swerveId == 0 || swerveId == 3 ? 0.29 : -0.29
            );
            STEER_OFFSET = steerOffset;
        }
    }

    public static final SwerveModuleConfigs FRONT_LEFT = new SwerveModuleConfigs(
        0,
        -0.65961173879082572877877766348154
    );

    public static final SwerveModuleConfigs FRONT_RIGHT = new SwerveModuleConfigs(
        1,
        0.06749515466696821410759585393765
    );

    public static final SwerveModuleConfigs BACK_LEFT = new SwerveModuleConfigs(
        2,
        -3.4775344461367486677709045653782
    );

    public static final SwerveModuleConfigs BACK_RIGHT = new SwerveModuleConfigs(
        3,
        -0.68108746982122470599483088973442
    );
}
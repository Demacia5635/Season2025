package frc.robot.chassis.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.chassis.ChassisConstants.SwerveModuleConfigs;
import frc.robot.utils.Cancoder;
import frc.robot.utils.TalonMotor;

public class SwerveModule {
    private TalonMotor steerMotor;
    private TalonMotor driveMotor;
    private Cancoder cancoder;
    VoltageOut voltageOutSteer;
    VoltageOut voltageOutDrive;

    public SwerveModule(SwerveModuleConfigs configs) {
        steerMotor = new TalonMotor(configs.STEER_CONFIG);
        driveMotor = new TalonMotor(configs.DRIVE_CONFIG);
        cancoder = new Cancoder(configs.CANCODER_CONFIG);

        voltageOutSteer = new VoltageOut(0);
        voltageOutDrive = new VoltageOut(0);

        steerMotor.setPosition(getAbsoluteAngle() - configs.STEER_OFFSET);

        SmartDashboard.putData(configs.DRIVE_CONFIG.name, driveMotor);
        SmartDashboard.putData(configs.STEER_CONFIG.name, steerMotor);
    }

    public double getSteerVolt() {
        return steerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getDriveVel() {
        return driveMotor.getCurrentVelocity();
    }

    public double getDriveAcc() {
        return driveMotor.getAcceleration().getValueAsDouble();
    }

    public double getDriveVolt() {
        return driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setSteerVolt(double volt) {
        steerMotor.setControl(voltageOutSteer.withOutput(volt));
    }

    public void setDriveVolt(double volt) {
        driveMotor.setControl(voltageOutDrive.withOutput(volt));
    }

    public void setSteerPower(double power) {
        steerMotor.set(power);
    }

    public double getAbsoluteAngle() {
        return cancoder.getCurrentAbsPosition();
    }

    public void setDrivePower(double power) {
        driveMotor.set(power);
    }

    public void setSteerVelocity(double velocityRadsPerSecond) {
        steerMotor.setVelocity(velocityRadsPerSecond);
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        driveMotor.setVelocity(velocityMetersPerSecond);
    }

    public void setSteerPosition(double positionRadians) {
        steerMotor.setPositionVoltage(positionRadians);
        // steerMotor.setMotionMagic(positionRadians);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(steerMotor.getCurrentPosition());
    }
    public double getSteerVel() {
        return steerMotor.getCurrentVelocity();
    }
    public double getSteerAccel() {
        return steerMotor.getAcceleration().getValueAsDouble();
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getSteerAngle());
        setDriveVelocity(state.speedMetersPerSecond);
        setSteerPosition(state.angle.getRadians());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getCurrentPosition(), Rotation2d.fromRadians(steerMotor.getCurrentPosition()));
    }
}

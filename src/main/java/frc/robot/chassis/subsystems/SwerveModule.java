package frc.robot.chassis.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.chassis.ChassisConstants.SwerveModuleConfigs;
import frc.robot.utils.Cancoder;
import frc.robot.utils.ConsoleAlert;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;
import static frc.robot.utils.Utils.*;

public class SwerveModule {
    private TalonMotor steerMotor;
    private TalonMotor driveMotor;
    private Cancoder cancoder;
    public String name;

    public SwerveModule(SwerveModuleConfigs configs) {
        steerMotor = new TalonMotor(configs.STEER_CONFIG);
        driveMotor = new TalonMotor(configs.DRIVE_CONFIG);
        cancoder = new Cancoder(configs.CANCODER_CONFIG);
        name = configs.NAME;

        steerMotor.setPosition(getAbsoluteAngle() - configs.STEER_OFFSET);

        SmartDashboard.putData(configs.DRIVE_CONFIG.name, driveMotor);
        SmartDashboard.putData(configs.STEER_CONFIG.name, steerMotor);
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

    public double getSteerAngle() {
        return steerMotor.getCurrentPosition();
    }
    public double getSteerVel() {
        return steerMotor.getCurrentVelocity();
    }
    public double getSteerAccel() {
        return steerMotor.getAcceleration().getValueAsDouble();
    }

    ConsoleAlert alert;

    public void setState(SwerveModuleState state) {
        if (alert == null) {
            alert = LogManager.log("messege");
        } 
        state.angle = new Rotation2d(MathUtil.angleModulus(state.angle.getRadians()));
        double currentAngle = steerMotor.getCurrentPosition();
        double delta = state.angle.minus(new Rotation2d(currentAngle)).getRadians();
        
        alert.setText("angle: " + state.angle + "\ndelta: " + delta);
    
        if (Math.abs(delta) > 0.5 * Math.PI) {
            if (delta > 0) {
                state.angle = Rotation2d.fromRadians(state.angle.getRadians() - Math.PI);
                state.speedMetersPerSecond = -state.speedMetersPerSecond;
                alert.setText("-");

            } else {
                state.angle = Rotation2d.fromRadians(state.angle.getRadians() + Math.PI);
                state.speedMetersPerSecond = -state.speedMetersPerSecond;
                alert.setText("+");

            }
        }
        else{
            alert.setText("nun");
        }

    
        setDriveVelocity(state.speedMetersPerSecond);
        setSteerPosition(state.angle.getRadians());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getCurrentPosition(), Rotation2d.fromRadians(steerMotor.getCurrentPosition()));
    }
}

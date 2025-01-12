package frc.robot.chassis.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.chassis.ChassisConstants;

public class Chassis extends SubsystemBase {
    private SwerveModule[] modules;
    private Pigeon2 gyro;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;

    public Chassis() {
        modules = new SwerveModule[] {
            new SwerveModule(ChassisConstants.FRONT_LEFT),
            new SwerveModule(ChassisConstants.FRONT_RIGHT),
            new SwerveModule(ChassisConstants.BACK_LEFT),
            new SwerveModule(ChassisConstants.BACK_RIGHT),
        };
        gyro = new Pigeon2(ChassisConstants.GYRO_ID, ChassisConstants.CANBus);

        kinematics = new SwerveDriveKinematics(
            ChassisConstants.FRONT_LEFT.POSITION,
            ChassisConstants.FRONT_RIGHT.POSITION,
            ChassisConstants.BACK_LEFT.POSITION,
            ChassisConstants.BACK_RIGHT.POSITION

        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
        field = new Field2d();

        SmartDashboard.putData("reset gyro", new InstantCommand(()-> poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), getPose())));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setDriveVolts(double volts) {
        for (SwerveModule i : modules) {
            i.setDriveVolt(volts);
        }
    }

    public void setSteerVolts(double volts) {
        for (SwerveModule i : modules) {
            i.setSteerVolt(volts);
        }
    }

    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setSteerPower(double pow, int id){
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id){
        return modules[id].getSteerVel();
    }
    public double getSteeracceleration(int id){
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position});
    }


    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity});
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRadians(gyro.getAngle());
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }

    private void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroAngle(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
        
    public boolean isRed() {
        return RobotContainer.isRed();
    }
}

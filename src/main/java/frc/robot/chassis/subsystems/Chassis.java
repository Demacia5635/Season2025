package frc.robot.chassis.subsystems;

import static frc.robot.vision.utils.VisionConstants.*;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.TrajectoryConstants.PathsConstraints;
import frc.robot.RobotContainer.AutoMode;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import static frc.robot.chassis.utils.ChassisConstants.*;
import frc.robot.chassis.utils.SwerveKinematics;
import frc.robot.utils.LogManager;
import frc.robot.vision.subsystem.Tag;
import frc.robot.vision.utils.VisionFuse;

public class Chassis extends SubsystemBase {
    private SwerveModule[] modules;
    private Pigeon2 gyro;
    private SwerveKinematics kinematicsFIx;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;
    private Field2d fieldTag;
    public Tag reefTag;
    public Tag fiderTag;
    public Tag bargeTag;
    public Tag backTag;
    public VisionFuse visionFuse;

    private StatusSignal<Angle> gyroYawStatus;
    private Rotation2d lastGyroYaw;

    public Chassis() {
        modules = new SwerveModule[] {
                new SwerveModule(FRONT_LEFT),
                new SwerveModule(FRONT_RIGHT),
                new SwerveModule(BACK_LEFT),
                new SwerveModule(BACK_RIGHT),
        };
        gyro = new Pigeon2(GYRO_ID, GYRO_CAN_BUS);
        addStatus();
        kinematicsFIx = new SwerveKinematics(
                FRONT_LEFT.POSITION,
                FRONT_RIGHT.POSITION,
                BACK_LEFT.POSITION,
                BACK_RIGHT.POSITION

        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematicsFIx, getGyroAngle(), getModulePositions(), new Pose2d());

        SimpleMatrix std = new SimpleMatrix(new double[] { 0.02, 0.02, 0 });
        poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(std));
        field = new Field2d();
        fieldTag = new Field2d();
        reefTag = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 0);
        fiderTag = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 1);
        bargeTag = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 2);
        backTag = new Tag(()->getGyroAngle(), ()->getChassisSpeedsRobotRel(), 3);
        visionFuse = new VisionFuse(reefTag, fiderTag, bargeTag, backTag);


        SmartDashboard.putData("reset gyro", new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("reset gyro 180", new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
        SmartDashboard.putData("set gyro to 3D tag", new InstantCommand(() -> setYaw(
                Rotation2d.fromDegrees(RobotContainer.robotContainer.autoChooser.getSelected() == AutoMode.MIDDLE
                        ? reefTag.getAngle()
                        : backTag.getAngle())))
                .ignoringDisable(true));
        LogManager.addEntry("gyro", () -> gyro.getYaw().getValueAsDouble());
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("ultfielf", fieldTag);
        LogManager.addEntry("VELOCITY NORM: ", () -> new Translation2d(getChassisSpeedsRobotRel().vxMetersPerSecond,
                getChassisSpeedsRobotRel().vyMetersPerSecond).getNorm());
        LogManager.addEntry("Chassis/vX", () -> getChassisSpeedsRobotRel().vxMetersPerSecond);
        LogManager.addEntry("Chassis/vY", () -> getChassisSpeedsRobotRel().vyMetersPerSecond);
        SmartDashboard.putData("Chassis/set coast", new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("Chassis/set brake", new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        SmartDashboard.putData(getName() + "/Swerve Drive", getChassisWidget());
        SmartDashboard.putData("Chassis", this);
    }

    private Sendable getChassisWidget() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAbsoluteAngle(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getDriveVel(), null);

                builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAbsoluteAngle(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getDriveVel(), null);

                builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAbsoluteAngle(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getDriveVel(), null);

                builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAbsoluteAngle(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getDriveVel(), null);

                builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getRadians(), null);
            }
        };
    }

    public void setNeutralMode(boolean isBrake) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(isBrake);
        }
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValueAsDouble());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /*public void setVelocitiesWithAccel(ChassisSpeeds speeds, boolean isPaths) {
        ChassisSpeeds limitedAccel = limitAccel(speeds, isPaths);

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(limitedAccel, getGyroAngle());

        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }*/
    
    

    Translation2d lastWantedSpeeds = new Translation2d();
    private void setVelocitiesWithAccel(ChassisSpeeds wantedSpeeds){
        Translation2d wantedVector = new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);
        Translation2d limitedVelocitiesVector = calculateVelocity(wantedVector, lastWantedSpeeds);
        ChassisSpeeds limitedVelocities = new ChassisSpeeds(limitedVelocitiesVector.getX(), limitedVelocitiesVector.getY(), wantedSpeeds.omegaRadiansPerSecond);
        limitedVelocities = ChassisSpeeds.discretize(limitedVelocities, CYCLE_DT);
        lastWantedSpeeds = limitedVelocitiesVector;
        setVelocities(limitedVelocities);

    }

    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());

        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    private double calculateLinearVelocity(Translation2d wantedSpeeds, Translation2d lastWantedSpeeds) {
        double deltaV = wantedSpeeds.getNorm() - lastWantedSpeeds.getNorm();
        if(Math.abs(deltaV) > AccelConstants.MAX_LINEAR_ACCEL * 0.02){
            return lastWantedSpeeds.getNorm() + (Math.signum(deltaV) * AccelConstants.MAX_LINEAR_ACCEL * 0.02);
        }
        return wantedSpeeds.getNorm();
    }
    private Translation2d calculateVelocity(Translation2d wantedSpeeds, Translation2d lastWantedSpeeds){
        double angleDiff = wantedSpeeds.getAngle().getRadians() - lastWantedSpeeds.getAngle().getRadians();
        double wantedLinearVelocity = calculateLinearVelocity(wantedSpeeds, lastWantedSpeeds);
        if(Math.abs(angleDiff) <= AccelConstants.MIN_OMEGA_DIFF)
            return wantedSpeeds.times(wantedLinearVelocity / wantedSpeeds.getNorm());

        double radius = wantedSpeeds.getNorm() / AccelConstants.MAX_OMEGA_VELOCITY;
        if(radius > AccelConstants.MAX_RADIUS){
            wantedLinearVelocity = AccelConstants.MAX_RADIUS * AccelConstants.MAX_OMEGA_VELOCITY;
             
        }
        return new Translation2d(wantedLinearVelocity,
            lastWantedSpeeds.getAngle().plus(
                new Rotation2d(AccelConstants.MAX_OMEGA_VELOCITY * CYCLE_DT * Math.signum(angleDiff))));

    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setSteerPower(double pow, int id) {
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id) {
        return modules[id].getSteerVel();
    }

    public double getSteeracceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position });
    }

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
    }

    public void setRobotRelVelocities(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity });
    }

    public Rotation2d getGyroAngle() {
        gyroYawStatus.refresh();
        if (gyroYawStatus.getStatus() == StatusCode.OK) {
            lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
        }
        return lastGyroYaw;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    private void updateVision(Pose2d pose) {
        poseEstimator.setVisionMeasurementStdDevs(getSTD());
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
    }

    private Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;

        Translation2d velocityVector = new Translation2d(getChassisSpeedsRobotRel().vxMetersPerSecond,
                getChassisSpeedsRobotRel().vyMetersPerSecond);
        double speed = velocityVector.getNorm();

        // Vision confidence adjustment
        if (visionFuse.getVisionConfidence() < 0.3) {
            x += 0.3;
            y += 0.3;
        }

        // Speed-based confidence calculation
        if (speed > WORST_RELIABLE_SPEED) {
            // Maximum uncertainty for high speeds
            x += 0.02;
            y += 0.02;
        } else if (speed <= BEST_RELIABLE_SPEED) {
            // Minimum uncertainty for low speeds
            x -= 0.02;
            y -= 0.02;
        } else {
            // Calculate normalized speed for the falloff range
            double normalizedSpeed = (speed - BEST_RELIABLE_SPEED)
                    / (WORST_RELIABLE_SPEED - BEST_RELIABLE_SPEED);

            // Apply exponential falloff to calculate additional uncertainty
            double speedConfidence = Math.exp(-3 * normalizedSpeed);

            // Scale the uncertainty adjustment based on confidence
            double adjustment = 0.02 * (1 - speedConfidence);
            x += adjustment;
            y += adjustment;
        }

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { x, y, theta }));
    }

    private boolean isSeeScoringTag() {
        return isSeeTag(RobotContainer.scoringTarget.position.getId(), 0, 3.5)
                || isSeeTag(RobotContainer.scoringTarget.position.getId(), 0, 3.5);

    }

    Pose2d visionFusePoseEstimation;

    @Override
    public void periodic() {
        visionFusePoseEstimation = visionFuse.getPoseEstemation();
        if (visionFusePoseEstimation != null) {
            updateVision(new Pose2d(visionFusePoseEstimation.getTranslation(), getGyroAngle()));
            // fieldTag.setRobotPose(visionFusePoseEstimation);
        }
        poseEstimator.update(getGyroAngle(), getModulePositions());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public boolean isRed() {
        return RobotContainer.isRed();
    }

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return kinematicsFIx.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(kinematicsFIx.toChassisSpeeds(getModuleStates()), getGyroAngle());
    }

    /**
     * Returns the state of every module
     * 
     * 
     * @return Velocity in m/s, angle in Rotation2d
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
            poseEstimator
                    .resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyro.getRotation2d()));
        }
    }

    public void setVelocitiesRotateToAngleOld(ChassisSpeeds speeds, Rotation2d angle, boolean isPaths) {
        double angleError = angle.minus(getGyroAngle()).getRadians();
        double angleErrorabs = Math.abs(angleError);
        if (angleErrorabs > Math.toRadians(1.5)) {
            speeds.omegaRadiansPerSecond = angleError * 2.3;
        }

        setVelocitiesWithAccel(speeds);
    }

    public void setVelocitiesRotateToTarget(ChassisSpeeds speeds, FieldTarget target) {
        Translation2d robotToTarget = target.getReefPole().getTranslation().minus(getPose().getTranslation());
        double angleError = robotToTarget.getAngle().minus(getGyroAngle()).getRadians();
        double angleErrorabs = Math.abs(angleError);
        if (angleErrorabs > Math.toRadians(1.5)) {
            speeds.omegaRadiansPerSecond = angleError * 2;
        }
        setVelocities(speeds);
    }

    PIDController drivePID = new PIDController(2, 0, 0);

    public void goTo(Pose2d pose, double threshold, boolean stopWhenFinished) {

        Translation2d diffVector = pose.getTranslation().minus(getPose().getTranslation());

        double distance = diffVector.getNorm();
        if (distance <= threshold) {
            if (stopWhenFinished)
                setVelocitiesRotateToAngleOld(new ChassisSpeeds(0, 0, 0), pose.getRotation(), true);
            else
                setVelocitiesRotateToAngleOld(
                        new ChassisSpeeds(0.5 * diffVector.getAngle().getCos(), 0.5 * diffVector.getAngle().getSin(),
                                0),
                        pose.getRotation(), true);
        }

        else {
            double vX = MathUtil.clamp(-drivePID.calculate(diffVector.getX(), 0), -3.2, 3.2);
            double vY = MathUtil.clamp(-drivePID.calculate(diffVector.getY(), 0), -3.2, 3.2);

            // LogManager.log("VX: " + vX + " VY: " + vY);

            setVelocitiesRotateToAngleOld(new ChassisSpeeds(vX, vY, 0), pose.getRotation(), true);
        }

    }

    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    public boolean isSeeTag(int id, int cameraId, double distance) {
        Tag[] tags = { reefTag, fiderTag, bargeTag, backTag };

        return tags[cameraId].isSeeTag(id, distance);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}

package frc.robot.chassis.subsystems;

import static frc.robot.vision.utils.VisionConstants.*;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.utils.ChassisConstants;
import frc.robot.chassis.utils.SwerveKinematics;
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
            new SwerveModule(ChassisConstants.FRONT_LEFT),
            new SwerveModule(ChassisConstants.FRONT_RIGHT),
            new SwerveModule(ChassisConstants.BACK_LEFT),
            new SwerveModule(ChassisConstants.BACK_RIGHT),
        };
        gyro = new Pigeon2(ChassisConstants.GYRO_ID, ChassisConstants.GYRO_CAN_BUS);
        addStatus();
        kinematicsFIx = new SwerveKinematics(
            ChassisConstants.FRONT_LEFT.POSITION,
            ChassisConstants.FRONT_RIGHT.POSITION,
            ChassisConstants.BACK_LEFT.POSITION,
            ChassisConstants.BACK_RIGHT.POSITION

        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematicsFIx, getGyroAngle(), getModulePositions(), new Pose2d());
        
        SimpleMatrix std = new SimpleMatrix(new double[]{0.02, 0.02, 0});
        poseEstimator.setVisionMeasurementStdDevs(new Matrix<>(std));
        field = new Field2d();
        fieldTag = new Field2d();
        reefTag = new Tag(()->getGyroAngle(), 0);
        fiderTag = new Tag(()->getGyroAngle(), 1);
        bargeTag = new Tag(()->getGyroAngle(), 2);
        backTag = new Tag(()->getGyroAngle(), 3);
        visionFuse = new VisionFuse(reefTag, fiderTag, bargeTag, backTag);

        SmartDashboard.putData("reset gyro", new InstantCommand(()-> setYaw(Rotation2d.fromDegrees(0))));
        SmartDashboard.putData("set gyro to 3D tag", new InstantCommand(()-> setYaw(visionFuse.getVisionEstimatedAngle())));
        SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("ultfielf", fieldTag);

    }

    public void resetPose(Pose2d pose){
        poseEstimator.resetPose(pose);
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValueAsDouble());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setVelocitiesAccelLim(ChassisSpeeds speeds) {
        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getGyroAngle());
        Translation2d wantedVelVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d currentVelVector = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        Rotation2d wantedVelAngle = wantedVelVector.getAngle();
        Rotation2d currentVelAngle = currentVelVector.getAngle();
        double wantedVelNorm = wantedVelVector.getNorm();
        double currentVelNorm = currentVelVector.getNorm();
        double newVel = wantedVelNorm;
        if(wantedVelNorm > currentVelNorm + ChassisConstants.DRIVE_MAX_VEL_CHNAGE){
            newVel = currentVelNorm + ChassisConstants.DRIVE_MAX_VEL_CHNAGE;
        } else if(wantedVelNorm < currentVelNorm - ChassisConstants.DRIVE_MAX_VEL_CHNAGE){
            newVel = currentVelNorm - ChassisConstants.DRIVE_MAX_VEL_CHNAGE;
        }
        double velForAngle = Math.max(ChassisConstants.MIN_DRIVE_VELOCITY_FOR_ROTATION, Math.abs(newVel));
        double maxAngleChange = ChassisConstants.DRIVE_ACCELERATION / velForAngle * ChassisConstants.CYCLE_DT;
        Rotation2d angleChange = wantedVelAngle.minus(currentVelAngle);
        Rotation2d newAngle = wantedVelAngle;
        if(angleChange.getRadians() > maxAngleChange){
            newAngle = currentVelAngle.plus(Rotation2d.fromRadians(maxAngleChange));
        } else if(angleChange.getRadians() < -maxAngleChange){
            newAngle = currentVelAngle.minus(Rotation2d.fromRadians(maxAngleChange));
        }
        wantedVelVector = new Translation2d(newVel, newAngle);
        ChassisSpeeds newSpeeds = new ChassisSpeeds(wantedVelVector.getX(), wantedVelVector.getY(), speeds.omegaRadiansPerSecond); 
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(newSpeeds, getGyroAngle());

        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        SwerveKinematics.desaturateWheelSpeeds(states, ChassisConstants.MAX_DRIVE_VELOCITY);
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

    public ChassisSpeeds getRobotRelVelocities(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), getGyroAngle());
    }
    public void setRobotRelVelocities(ChassisSpeeds speeds){
        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
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
    
    private void updateVision(Pose2d pose){
        poseEstimator.setVisionMeasurementStdDevs(getSTD());
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
    }

    private Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;
        
        Translation2d velocityVector = new Translation2d(getChassisSpeeds().vxMetersPerSecond, 
                                                       getChassisSpeeds().vyMetersPerSecond);
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
    
        return new Matrix<N3, N1>(new SimpleMatrix(new double[]{x, y, theta}));
    }

    Pose2d visionFusePoseEstimation;

    @Override
    public void periodic() {
        visionFusePoseEstimation = visionFuse.getPoseEstemation();
        if(visionFusePoseEstimation != null){
            updateVision(new Pose2d(visionFusePoseEstimation.getTranslation(), getGyroAngle()));
            // fieldTag.setRobotPose(visionFusePoseEstimation);
        }
        poseEstimator.update(getGyroAngle(), getModulePositions());
            
        
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        RobotContainer.feedingTarget.position = getPose().getY() > 4 ? POSITION.FEEDER_RIGHT : POSITION.FEEDER_LEFT;
    }
        
    public boolean isRed() {
        return RobotContainer.isRed();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematicsFIx.toChassisSpeeds(getModuleStates());
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
    if (angle != null){
        gyro.setYaw(angle.getDegrees());
        poseEstimator.resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyro.getRotation2d()));
    }
  }



  PIDController rotationPid = new PIDController(0.9, 0.1, 0);
  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle) {
    Rotation2d angleToUse = angle == null ? new Rotation2d() : angle;
    double angleError = angleToUse.minus(getGyroAngle()).getRadians();
    if (Math.abs(angleError)>Math.toRadians(1)){
        speeds.omegaRadiansPerSecond = rotationPid.calculate(getPose().getRotation().getRadians(), angleToUse.getRadians());
    }
    else{
        speeds.omegaRadiansPerSecond = 0;
    }
    setVelocities(speeds);
  }

  public void setVelocitiesRotateToAngleOld(ChassisSpeeds speeds, Rotation2d angle) {
    double angleError = angle.minus(getGyroAngle()).getRadians();
    double angleErrorabs = Math.abs(angleError);
    if (angleErrorabs>Math.toRadians(1.5)){
        speeds.omegaRadiansPerSecond = angleError * 1;
    }
    setVelocities(speeds);
  }

  
  PIDController drivePID = new PIDController(1.7, 0.1, 0);
  public void goTo(Pose2d pose, double threshold){
    
    Translation2d diffVector = pose.getTranslation().minus(getPose().getTranslation());

    
    double distance = diffVector.getNorm();
    if(distance <= threshold) stop();
    else{
        double vX = drivePID.calculate(-diffVector.getX(), 0);
        double vY = drivePID.calculate(-diffVector.getY(), 0);

        setVelocitiesRotateToAngle(new ChassisSpeeds(vX, vY, 0), pose.getRotation());
    }
    

  }



    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }


    public boolean isSeeTag(int id, int cameraId, double distance){
        Tag[] tags = {reefTag, fiderTag, bargeTag, backTag};
        
        return tags[cameraId].isSeeTag(id, distance);
    }
}

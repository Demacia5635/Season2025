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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.utils.ChassisConstants;
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
    private double driveAccel;
    private double pathsAccel;

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
        this.pathsAccel = ChassisConstants.AccelPaths.DEFAULT;
        this.driveAccel = ChassisConstants.AccelDrive.DEFAULT;

        SmartDashboard.putData("reset gyro", new InstantCommand(()-> setYaw(Rotation2d.fromDegrees(0))));
        SmartDashboard.putData("set gyro to 3D tag", new InstantCommand(()-> setYaw(visionFuse.getVisionEstimatedAngle())));
        SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("ultfielf", fieldTag);
        LogManager.addEntry("VELOCITY NORM: ", ()->new Translation2d(getChassisSpeedsRobotRel().vxMetersPerSecond, getChassisSpeedsRobotRel().vyMetersPerSecond).getNorm());
        LogManager.addEntry("Chassis vX", ()->getChassisSpeedsRobotRel().vxMetersPerSecond);
        
        LogManager.addEntry("Chassis vY", ()->getChassisSpeedsRobotRel().vyMetersPerSecond);
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


    public void setVelocitiesWithAccel(ChassisSpeeds speeds, boolean isPaths){
        ChassisSpeeds limitedAccel = limitAccel(speeds, isPaths);


        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(limitedAccel, getGyroAngle());
        
        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }
    public void setVelocities(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        
        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    ChassisSpeeds currentVelocity = new ChassisSpeeds();
    public ChassisSpeeds limitAccel(ChassisSpeeds wantedSpeeds, boolean isPaths) {
        
        double maxDeltaVel = adjustAccel(isPaths) * 0.02;
        LogManager.log("ACCEL: " + maxDeltaVel / 0.02);

        currentVelocity = getChassisSpeedsFieldRel();
        double vx = wantedSpeeds.vxMetersPerSecond;
        double vy = wantedSpeeds.vyMetersPerSecond;
        double deltaX = wantedSpeeds.vxMetersPerSecond - currentVelocity.vxMetersPerSecond;
        double deltaY = wantedSpeeds.vyMetersPerSecond - currentVelocity.vyMetersPerSecond;
        if(Math.abs(deltaX) > maxDeltaVel) vx = currentVelocity.vxMetersPerSecond + (maxDeltaVel * Math.signum(deltaX));
        if(Math.abs(deltaY) > maxDeltaVel) vy = currentVelocity.vyMetersPerSecond + (maxDeltaVel * Math.signum(deltaY));
        return new ChassisSpeeds(vx, vy, wantedSpeeds.omegaRadiansPerSecond);
    }
    private double adjustAccel(boolean isPaths){
        switch (RobotContainer.arm.state) {
            case L2:
                driveAccel = ChassisConstants.AccelDrive.L2;
                pathsAccel = ChassisConstants.AccelPaths.L2;
                break;
            case L3:
            
                driveAccel = ChassisConstants.AccelDrive.L3;
                pathsAccel = ChassisConstants.AccelPaths.L3;
                break;
            
            case ALGAE_TOP:
                
                driveAccel = ChassisConstants.AccelDrive.ALGAE_TOP;
                pathsAccel = ChassisConstants.AccelPaths.ALGAE_TOP;
                break;

            case ALGAE_BOTTOM:
                driveAccel = ChassisConstants.AccelDrive.ALGAE_BOTTOM;
                pathsAccel = ChassisConstants.AccelPaths.ALGAE_BOTTOM;
                break;

            case CORAL_STATION:
            
                driveAccel = ChassisConstants.AccelDrive.INTAKE;
                pathsAccel = ChassisConstants.AccelPaths.INTAKE;
                break;
            
            default:
            
                driveAccel = ChassisConstants.AccelDrive.DEFAULT;
                pathsAccel = ChassisConstants.AccelPaths.DEFAULT;
                break;
        }
        return isPaths ? pathsAccel : driveAccel;
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
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
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

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return kinematicsFIx.toChassisSpeeds(getModuleStates());
    }
    public ChassisSpeeds getChassisSpeedsFieldRel(){
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
    if (angle != null){
        gyro.setYaw(angle.getDegrees());
        poseEstimator.resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), gyro.getRotation2d()));
    }
  }


  public void setVelocitiesRotateToAngleOld(ChassisSpeeds speeds, Rotation2d angle, boolean isPaths) {
    double angleError = angle.minus(getGyroAngle()).getRadians();
    double angleErrorabs = Math.abs(angleError);
    if (angleErrorabs>Math.toRadians(1.5)){
        speeds.omegaRadiansPerSecond = angleError * 2.5;
    }

    setVelocitiesWithAccel(speeds, isPaths);
  }

  public void setVelocitiesRotateToTarget(ChassisSpeeds speeds, FieldTarget target) {
    Translation2d robotToTarget = target.getReefPole().getTranslation().minus(getPose().getTranslation());
    double angleError = robotToTarget.getAngle().minus(getGyroAngle()).getRadians();
    double angleErrorabs = Math.abs(angleError);
    if (angleErrorabs>Math.toRadians(1.5)){  
        speeds.omegaRadiansPerSecond = angleError * 2;
    }
    setVelocities(speeds);
  }

  
  PIDController drivePID = new PIDController(2.2, 0.7, 0);
  public void goTo(Pose2d pose, double threshold, boolean stopWhenFinished){
    
    Translation2d diffVector = pose.getTranslation().minus(getPose().getTranslation());

    
    double distance = diffVector.getNorm();
    if(distance <= threshold) {
        if(stopWhenFinished) setVelocitiesRotateToAngleOld(new ChassisSpeeds(0, 0, 0), pose.getRotation(), true);
        else setVelocitiesRotateToAngleOld(new ChassisSpeeds(1 * diffVector.getAngle().getCos(), 1 * diffVector.getAngle().getSin(), 0), pose.getRotation(), true);
    }
        
    else{
        double vX = drivePID.calculate(-diffVector.getX(), 0);
        double vY = drivePID.calculate(-diffVector.getY(), 0);

        setVelocitiesRotateToAngleOld(new ChassisSpeeds(vX, vY, 0), pose.getRotation(), true);
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

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}

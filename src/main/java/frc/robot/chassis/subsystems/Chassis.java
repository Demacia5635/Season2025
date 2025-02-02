package frc.robot.chassis.subsystems;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
        reefTag = new Tag(()->getGyroAngle(), 0);
        fiderTag = new Tag(()->getGyroAngle(), 1);
        bargeTag = new Tag(()->getGyroAngle(), 2);
        backTag = new Tag(()->getGyroAngle(), 3);
        visionFuse = new VisionFuse(reefTag, fiderTag, bargeTag, backTag);

        SmartDashboard.putData("reset gyro", new InstantCommand(()-> setYaw(Rotation2d.fromDegrees(0))));
        SmartDashboard.putData("set gyro to 3D tag", new InstantCommand(()-> setYaw(visionFuse.getVisionEstimatedAngle())));
        SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putData("field", field);

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

    public void setVelocitiesAccelLim(ChassisSpeeds speeds, double maxDriveAccel, double maxRotAccel) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle());
        ChassisSpeeds currentSpeeds = getChassisSpeeds();

        double dvx = speeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond;
        double dvy = speeds.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond;
        double domega = speeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond;
        
        dvx = Math.max(Math.min(dvx, maxDriveAccel), -maxDriveAccel);
        dvy = Math.max(Math.min(dvy, maxDriveAccel), -maxDriveAccel);
        domega = Math.max(Math.min(domega, maxRotAccel), -maxRotAccel);

        speeds = new ChassisSpeeds(
            currentSpeeds.vxMetersPerSecond + dvx,
            currentSpeeds.vyMetersPerSecond + dvy,
            currentSpeeds.omegaRadiansPerSecond + domega
        );

        SwerveModuleState[] states = kinematicsFIx.toSwerveModuleStates(speeds);
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
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - visionFuse.getVisionTimestamp());
    }

    private Matrix getSTD(){
        double x = 0.05;
        double y = 0.05;
        double theta = 0.05;
        Translation2d velocityVector = new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
        if(velocityVector.getNorm() <= 1.5){
            x-= 0.02;
            y-=0.02;
            theta -= 0.02;
        }
        if (velocityVector.getNorm() > 3.5) {
           x+= 0.02;
           y+= 0.02;
           theta += 0.02;
        }
        if (Math.abs(getChassisSpeeds().omegaRadiansPerSecond) >= 4) {
            theta += 0.02;
        }
        if(Math.abs(getChassisSpeeds().omegaRadiansPerSecond) <= 1){
            theta -= 0.02;
        }

        return new Matrix(new SimpleMatrix(new double[]{x, y, theta}));
    }

    @Override
    public void periodic() {

        //poseEstimator.setVisionMeasurementStdDevs(getSTD());
        if(visionFuse.getPoseEstemation() !=null){
            updateVision(new Pose2d(visionFuse.getPoseEstemation().getTranslation(), getGyroAngle()));
        }
        poseEstimator.update(getGyroAngle(), getModulePositions());
            
        
        field.setRobotPose(poseEstimator.getEstimatedPosition());
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



  PIDController rotationPid = new PIDController(2.3, 0.35, 0);
  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle) {
    double angleError = angle.minus(getGyroAngle()).getRadians();
    if (Math.abs(angleError)>Math.toRadians(1)){
        speeds.omegaRadiansPerSecond = rotationPid.calculate(getPose().getRotation().getRadians(), angle.getRadians());
    }
    else{
        speeds.omegaRadiansPerSecond = 0;
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

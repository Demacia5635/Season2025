package frc.robot.chassis.subsystems;

import frc.robot.utils.LogManager;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.chassis.constants.ChassisConstants;
import frc.robot.vision.subsystem.Tag;

public class Chassis extends SubsystemBase {
    private SwerveModule[] modules;
    private Pigeon2 gyro;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;
    public Tag reefTag;
    public Tag fiderTag;
    public Tag bargeTag;
    public Tag backTag;



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
        kinematics = new SwerveDriveKinematics(
            ChassisConstants.FRONT_LEFT.POSITION,
            ChassisConstants.FRONT_RIGHT.POSITION,
            ChassisConstants.BACK_LEFT.POSITION,
            ChassisConstants.BACK_RIGHT.POSITION

        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
        field = new Field2d();
        reefTag = new Tag(()->getGyroAngle(), 0);
        fiderTag = new Tag(()->getGyroAngle(), 1);
        bargeTag = new Tag(()->getGyroAngle(), 2);
        backTag = new Tag(()->getGyroAngle(), 3);

        SmartDashboard.putData("reset gyro", new InstantCommand(()-> setGyroAngle(Rotation2d.fromDegrees(0))));
        SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putData("goToVision", new InstantCommand(()-> poseEstimator.resetPose(getVisionEstematedPose())));
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

    public ChassisSpeeds getRobotRelVelocities(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeeds(), getGyroAngle());
    }
    public void setRobotRelVelocities(ChassisSpeeds speeds){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
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
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
    }
    @Override
    public void periodic() {

        if(getVisionEstematedPose() !=null){
            updateVision(new Pose2d(getVisionEstematedPose().getTranslation(), getGyroAngle()));
        }
        poseEstimator.update(getGyroAngle(), getModulePositions());
            
        
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
        
    public boolean isRed() {
        return RobotContainer.isRed();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
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

  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle) {
    double angleError = angle.minus(getGyroAngle()).getRadians();
    double angleErrorabs = Math.abs(angleError);
    if (angleErrorabs>Math.toRadians(1)){
        speeds.omegaRadiansPerSecond = angleError * 3;
    }
    setVelocities(speeds);
  }

  public void setGyroAngle(Rotation2d angle) {
    if (angle != null){
        gyro.setYaw(angle.getDegrees());
    }
  }


    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    public Pose2d getVisionEstematedPose() {
        // Array of all tags and their poses/confidences
        Tag[] tags = {reefTag, fiderTag, bargeTag, backTag};
        Pose2d bestPose = null;
        double highestConfidence = 0.0;
    
        // Find the tag with highest confidence
        for (Tag tag : tags) {
            Pose2d currentPose = tag.getPose();
            double currentConfidence = tag.getPoseEstemationConfidence();
    
            // Only consider poses with confidence above minimum threshold
            if (currentPose != null && currentConfidence > 0.1) {  // 10% minimum confidence threshold
                if (currentConfidence > highestConfidence) {
                    highestConfidence = currentConfidence;
                    bestPose = currentPose;
                }
            }
        }
    
        return bestPose;  // Will return null if no tags meet confidence threshold
    }
}

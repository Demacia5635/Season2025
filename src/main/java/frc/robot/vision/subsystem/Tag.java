package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.vision.utils.VisionConstants.*;

import java.util.function.Supplier;

/**
 * Subsystem for processing AprilTag vision data and calculating robot position.
 * Uses multiple Limelight cameras and a Pigeon2 gyro to determine robot position on field.
 */
public class Tag extends SubsystemBase {

    private Translation2d robotToTag;
    private Translation2d cameraToTag;
    private double alpha;

    // NetworkTables communication for each camera
    private NetworkTable ltagTable = NetworkTableInstance.getDefault().getTable(TABLE[0]);
    private NetworkTable rtagTable = NetworkTableInstance.getDefault().getTable(TABLE[1]);
    private NetworkTable stagTable = NetworkTableInstance.getDefault().getTable(TABLE[2]);

    private NetworkTable[] Tables = {ltagTable, rtagTable, stagTable};

    // Vision processing variables
    private double camToTagYaw;
    private double camToTagPitch;
    private double id;
    private double height;
    private double dist;
    private Translation2d originToRobot;
    private Translation2d origintoTag;
    private Translation2d robotToTagRR;
    private Rotation2d LLAngle;
    private int camId;
    private Translation2d[] RobotsToTagsRR;
    private double[] ids;
    private double count;

    private Translation2d T1;
    private Translation2d T2;
    private double ID1;
    private double ID2;

    private Translation2d TagToTagRR;
    private Translation2d TagToTagFR;

    private NetworkTableEntry cropEntry;

    
    private Supplier<Rotation2d> getRobotAngle;
    private Field2d field;

    /**
     * Creates a new Tag subsystem
     * @param robot_angle_from_pose Pigeon2 gyroscope for determining robot orientation
     */
    public Tag(Supplier<Rotation2d> robot_angle_from_pose) {
        this.getRobotAngle = robot_angle_from_pose;

        // Initialize NetworkTables connections for all cameras
        ltagTable = NetworkTableInstance.getDefault().getTable(TABLE[0]);
        rtagTable = NetworkTableInstance.getDefault().getTable(TABLE[1]);
        stagTable = NetworkTableInstance.getDefault().getTable(TABLE[2]);


        field = new Field2d();
        SmartDashboard.putData("Tag", this);
        SmartDashboard.putData("field-tag", field);
    }

    @Override
    public void periodic() {
        // Initialize arrays if they haven't been initialized
        ids = new double[Tables.length];
        RobotsToTagsRR = new Translation2d[Tables.length];
        
        // Reset camId to -1
        camId = 0;
        
        // Process data from each camera
        for (NetworkTable t : Tables) {
            if (t.getEntry("tv").getDouble(0.0) != 0) {
                camToTagPitch = t.getEntry("ty").getDouble(0.0);
                camToTagYaw = -t.getEntry("tx").getDouble(0.0);
                id = t.getEntry("tid").getDouble(0.0);
                cropEntry = t.getEntry("crop");
                crop();
                // Only process valid tag IDs
                if (id > 0 && id < TAG_HIGHT.length) {
                    ids[camId] = id;
                    RobotsToTagsRR[camId] = getRobotToTagRR(camId);
                }
            }
            else{
              cropStop();
            }
            camId++;
        }
        
        if (visibleTags(ids) > 1) {
            // add multi-tag angle estimation
            // Calculate angle using multiple tags
            LLAngle = calcAngle();
            // Use first valid tag for position
            for (int i = 0; i < ids.length; i++) {
                if (ids[i] > 0) {
                    Pose2d pose = new Pose2d(getOriginToRobot(i, LLAngle), LLAngle);
                    field.setRobotPose(pose);
                    break;
                }
            }
        } else if (visibleTags(ids) == 1) {
            // Use single tag with gyro angle
            for (int i = 0; i < ids.length; i++) {
                if (ids[i] > 0) {
                    Pose2d pose = new Pose2d(getOriginToRobot(i, getRobotAngle.get()), getRobotAngle.get());
                    field.setRobotPose(pose);
                    break;
                }
            }
        }
        // If no tags visible, pose is not updated
    }

      /**
     * Calculates straight-line distance from camera to AprilTag
     * Uses trigonometry with known tag height and camera angle
     * @return Distance in meters
     */
    public double GetDistFromCamera(int cam) {
      alpha = camToTagPitch + CAM_PITHC[cam];
      dist = (Math.abs(height - CAM_HIGHT[cam])) / (Math.tan(Math.toRadians(alpha)));
      dist = dist/Math.cos(Math.toRadians(camToTagYaw));
      return dist;
    }

      /**
     * Calculates vector from robot center to detected AprilTag
     * Accounts for camera offset from robot center
     * @return Translation2d representing vector to tag
     */
    public Translation2d getRobotToTagRR(int cam) {
      // Convert camera measurements to vector
      cameraToTag = new Translation2d(GetDistFromCamera(cam), 
          Rotation2d.fromDegrees(camToTagYaw));
          
      // Add camera offset to get robot center to tag vector
      robotToTag = ROBOT_TO_CAM[cam].plus(cameraToTag);
      robotToTag = robotToTag.rotateBy(Rotation2d.fromDegrees(CAM_YAW[cam]));

      return robotToTag;
  }

  /**
     * Calculates robot position relative to field origin
     * Uses known AprilTag position and measured vector to tag
     * @return Translation2d representing robot position on field
     */
    public Translation2d getOriginToRobot(int cam, Rotation2d Angle) {

      origintoTag = O_TO_TAG[(int)this.id];

      height = TAG_HIGHT[(int)this.id];
      if(origintoTag != null) {
          // Get vector from robot to tag
          robotToTagRR = getRobotToTagRR(cam);

          Translation2d robotToTagFC = robotToTagRR.rotateBy(Angle);
                    originToRobot = origintoTag.plus(robotToTagFC.rotateBy(Rotation2d.fromDegrees(180)));
          
                    return originToRobot;
                }
                return new Translation2d();
                
      }
          
      private Rotation2d calcAngle() {
        T1 = null;
        T2 = null;
        for (Translation2d RTT : RobotsToTagsRR) {
          if(RTT != null){
            if(T1 == null){
              T1 = RTT;
            }
            else if(T2 == null){
              T2 = RTT;
            }
          }
        }
        ID1 = 0;
        ID2 = 0;
        for (double id : ids) {
          if(id != 0){
            if(ID1 == 0){
              ID1 = id;
            }
            else if(ID2 == 0){
              ID2 = id;
            }
          }
        }
        // robot relativ tag vector
        TagToTagRR = T1.minus(T2);
        //field relativ tag vector
        TagToTagFR = O_TO_TAG[(int)ID1];
        // calc the difrenc
        LLAngle = TagToTagRR.getAngle().minus(TagToTagFR.getAngle());
        return LLAngle;
      }

      private double visibleTags(double[] ids){
        count = 0;
        for (double id : ids) {
          count = (id == 0)?count:count+1;
        }
        return count;
      }
      private void crop(){
        double YawCrop = camToTagYaw/31.25;
        double PitchCrop = camToTagPitch/24.45;
        double[] crop = {YawCrop-CROP_OFSET,YawCrop+CROP_OFSET,PitchCrop-CROP_OFSET,PitchCrop+CROP_OFSET};
        cropEntry.setDoubleArray(crop);
      }
      private void cropStop(){
        double[] crop = {-1,1,-1, 1};
        cropEntry.setDoubleArray(crop);
      }
}
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
import frc.robot.utils.LogManager;

import static frc.robot.vision.VisionConstants.*;

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
    private NetworkTable Table;

    // Vision processing variables
    private double camToTagYaw;
    private double camToTagPitch;
    private double id;
    private double height;
    private double dist;
    private Translation2d originToRobot;
    private Translation2d origintoTag;
    private Translation2d robotToTagRR;
    public Pose2d pose;

    private NetworkTableEntry cropEntry;
   
    private Supplier<Rotation2d> getRobotAngle;
    private Field2d field;

    private double latency;

    private Translation2d robotToTagFC;

    private double tagID = 0;

    private double Yaw3d;
    private Rotation2d yaw3dRotation2d;

    private int cameraId;

    private double confidence = 0;

    /**
     * Creates a new Tag subsystem
     * @param robot_angle_from_pose Pigeon2 gyroscope for determining robot orientation
     */
    public Tag(Supplier<Rotation2d> robot_angle_from_pose, int cameraId) {
        this.getRobotAngle = robot_angle_from_pose;
        this.cameraId = cameraId;
      
        Table = NetworkTableInstance.getDefault().getTable(TABLE[cameraId]);

        field = new Field2d();
        latency = 0;
        SmartDashboard.putData("Tag" + cameraId, this);
        SmartDashboard.putData("field-tag" + cameraId, field);
    }


    @Override
    public void periodic() {
      // Process data from each camera
      cropEntry = Table.getEntry("crop");
      camToTagPitch = Table.getEntry("ty").getDouble(0.0);
      camToTagYaw = (-Table.getEntry("tx").getDouble(0.0));// + CAM_YAW[cameraId];
      id = Table.getEntry("tid").getDouble(0.0);

      latency = Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0);

      if (Table.getEntry("tv").getDouble(0.0) != 0) {
          crop();
          // Only process valid tag IDs
          if (id > 0 && id < TAG_HIGHT.length) {
            pose = new Pose2d(getOriginToRobot(getRobotAngle.get()), getRobotAngle.get());
            field.setRobotPose(pose);
            confidence = getConfidence();
          }
      }
      else{
        cropStop();
        pose = null;
      }
    }


      /**
     * Calculates straight-line distance from camera to AprilTag
     * Uses trigonometry with known tag height and camera angle
     * @return Distance in meters
     */
    public double GetDistFromCamera() {
      if (cameraId == 0 || cameraId == 3){
        alpha = camToTagPitch + CAM_PITHC[cameraId];
        dist = (Math.abs(height - CAM_HIGHT[cameraId])) * (Math.tan(Math.toRadians(alpha)));
        //dist = dist/Math.cos(Math.toRadians(camToTagYaw));
        return Math.abs(dist);
      }
      alpha = camToTagPitch + CAM_PITHC[cameraId];
      dist = (Math.abs(height - CAM_HIGHT[cameraId])) / (Math.tan(Math.toRadians(alpha)));
      //dist = dist/Math.cos(Math.toRadians(camToTagYaw));
      return dist;
    }


      /**
     * Calculates vector from robot center to detected AprilTag
     * Accounts for camera offset from robot center
     * @return Translation2d representing vector to tag
     */
    public Translation2d getRobotToTagRR() {
      // Convert camera measurements to vector
      cameraToTag = new Translation2d(GetDistFromCamera(),
          Rotation2d.fromDegrees(camToTagYaw));
         
      // Add camera offset to get robot center to tag vector
      robotToTag = ROBOT_TO_CAM[cameraId].plus(cameraToTag);
      robotToTag = robotToTag.rotateBy(Rotation2d.fromDegrees(CAM_YAW[cameraId]));
      


      return robotToTag;
    }




  /**
     * Calculates robot position relative to field origin
     * Uses known AprilTag position and measured vector to tag
     * @return Translation2d representing robot position on field
     */
    public Translation2d getOriginToRobot(Rotation2d Angle) {


      origintoTag = O_TO_TAG[(int)this.id == -1 ? 0 : (int)this.id];


      height = TAG_HIGHT[(int)this.id == -1 ? 0 : (int)this.id];
      if(origintoTag != null) {
        // Get vector from robot to tag
        robotToTagRR = getRobotToTagRR();


        robotToTagFC = robotToTagRR.rotateBy(Angle);
        originToRobot = origintoTag.plus(robotToTagFC.rotateBy(Rotation2d.fromDegrees(180)));

       
        return originToRobot;
      }
        return new Translation2d();
               
      }
      private void crop(){
        double YawCrop = ((-camToTagYaw))/31.25;
        double PitchCrop = camToTagPitch/24.45;
        double[] crop = {YawCrop-CROP_OFSET,YawCrop+CROP_OFSET,PitchCrop-CROP_OFSET,PitchCrop+CROP_OFSET};
        cropEntry.setDoubleArray(crop);
      }
      private void cropStop(){
        double[] crop = {-1,1,-1, 1};
        cropEntry.setDoubleArray(crop);
      }
      public Pose2d getPose(){
        return this.pose;
      }


      public double getTimestamp() {
        return latency;
      }
      public Rotation2d getRobotAngle(){
        Table.getEntry("pipeline").setNumber(1);
        try{
          Yaw3d = Table.getEntry("camerapose_targetspace").getDoubleArray(new double[]{0,0,0,0,0,0})[4];
          tagID = Table.getEntry("tid").getDouble(0.0);
          Table.getEntry("pipeline").setNumber(0);
          yaw3dRotation2d = Rotation2d.fromDegrees(Yaw3d).rotateBy(Rotation2d.fromDegrees(CAM_YAW[cameraId])).rotateBy(TAG_ANGLE[(int)tagID]).rotateBy(Rotation2d.fromDegrees(180));
          return yaw3dRotation2d;

        }catch(Exception E){
          getRobotAngle();
        }

      return null;
    }

    public double getPoseEstemationConfidence(){
      return this.confidence;
    }

    private double getConfidence() {
      // Get the current distance to tag
      double currentDist = GetDistFromCamera();
      
      // Constants for confidence calculation
      final double MAX_RELIABLE_DISTANCE = 1; // meters - high confidence range
      final double MAX_DETECTION_DISTANCE = 4; // meters - maximum useful detection range
      
      // If we're too far, return 0 confidence
      if (currentDist > MAX_DETECTION_DISTANCE) {
          return 0.0;
      }
      
      // If we're within reliable range, give high confidence
      if (currentDist <= MAX_RELIABLE_DISTANCE) {
          return 1.0;
      }
      
      // Calculate how far we are into the falloff range (0 to 1)
      double normalizedDist = (currentDist - MAX_RELIABLE_DISTANCE) 
                             / (MAX_DETECTION_DISTANCE - MAX_RELIABLE_DISTANCE);
      
      // Apply cubic falloff function
      return Math.pow(1 - normalizedDist, 3);
  }

  public boolean isSeeTag(int id, double distance){
    return Table.getEntry("tid").getDouble(0.0) == id && getRobotToTagRR().getNorm() <= distance;
  }
}


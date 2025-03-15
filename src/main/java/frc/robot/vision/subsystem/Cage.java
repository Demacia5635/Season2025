// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import java.net.NetworkInterface;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Camera;
import frc.robot.vision.Camera.CameraType;
import frc.robot.vision.utils.VisionConstants;

import static frc.robot.vision.utils.VisionConstants.*;


public class Cage extends SubsystemBase {
  
  private Camera camera;
  private NetworkTable Table;
  private NetworkTableEntry pipeEntry;


  private Pose2d pose;
  private double alpha;
  private double camToTagPitch;
  private double height = VisionConstants.BARGE_HIGHT_CENTER;
  

  private Supplier<Rotation2d> getRobotAngle;
  private Field2d field;

  private Translation2d cageToCam;
  private Translation2d cageToRobot;
  private Translation2d cageToOrigin;
  private Supplier<Pose2d> robotToOrigin;

  private double camToCageYaw;
  private Translation2d cageToRobotFC;


  private double dist;


  public Cage(Supplier<Pose2d> poseSupplier,Camera camera) {
      this.camera = camera;
      this.robotToOrigin = poseSupplier;

      Table = NetworkTableInstance.getDefault().getTable(camera.getName());
      field = new Field2d();
      SmartDashboard.putData("field-tag" + camera.getName(), field);

  }

  public double GetDistFromCamera() {

    alpha = camToTagPitch + camera.getPitch();
    dist = (Math.abs(height - camera.getHeight())) / (Math.tan(Math.toRadians(alpha)));
    dist = dist/Math.cos(Math.toRadians(camToCageYaw));
    return Math.abs(dist);
  }
    public Translation2d getRobotToCage() {
    // Convert camera measurements to vector
    cageToCam = new Translation2d(GetDistFromCamera(),
        Rotation2d.fromDegrees(camToCageYaw));
    // LogManager.log("cameraToTag :" +cameraToTag);
    // LogManager.log("Camera to Tag Yaw :" + camToTagYaw);
    // Add camera offset to get robot center to tag vector
    cageToRobot = new Translation2d(camera.getRobotToCamPosition().getX(), camera.getRobotToCamPosition().getY())
      .plus(cageToCam);
    // LogManager.log("Robot to Tag :" + robotToTag);
    return cageToRobot;
  }
  public Translation2d getOriginToCage(){
    cageToRobot = getRobotToCage();

    if(cageToRobot != null){
      cageToRobotFC = cageToRobot.rotateBy(getRobotAngle.get());
      cageToOrigin = robotToOrigin.get().getTranslation().plus(cageToRobotFC.rotateBy(Rotation2d.kPi));
  
      return cageToOrigin;
    }

    return new Translation2d();
  }

  public Translation2d getCageToCamera() {
    return new Translation2d(GetDistFromCamera(),
        Rotation2d.fromDegrees(camToCageYaw));
  }
  public Pose2d getPose2d(){
    return this.pose;
  }

  @Override
  public void periodic() {
    pipeEntry = Table.getEntry("needToCange");//TODO:cange to right pipeline
    camToTagPitch = Table.getEntry("ty").getDouble(0.0);
    camToCageYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();

    if (Table.getEntry("tv").getDouble(0.0) != 0) {

      // Only process valid tag IDs
      pose = new Pose2d(getOriginToCage(), getRobotAngle.get());
      field.setRobotPose(pose);
    }
  }
}

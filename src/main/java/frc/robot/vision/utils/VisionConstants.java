// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants and configuration values for AprilTag vision processing.
 * This class contains the physical layout and properties of AprilTags on the field,
 * as well as camera mounting parameters.
 */
public class VisionConstants {

    // Heights of different AprilTag groups from the ground
    private static double BARGE_TAG_HIGHT = inchToMeter(73.54);
    private static double REEF_TAG_HIGHT = inchToMeter(12.13);
    private static double STATION_TAG_HIGHT = inchToMeter(58.50);
    private static double SIDE_TAG_HIGHT = inchToMeter(51.25);

    /**
     * Array of AprilTag positions relative to field origin (0,0).
     * Each Translation2d represents X,Y coordinates in meters.
     * Index corresponds to AprilTag ID (0 is unused).
     * Coordinates are converted from inches to meters using inchToMeter().
     */
    public static Translation2d[] O_TO_TAG = {null,//0
        new Translation2d(inchToMeter(657.37), inchToMeter(25.80)),//1
        new Translation2d(inchToMeter(657.37), inchToMeter(291.20)),//2
        new Translation2d(inchToMeter(455.15), inchToMeter(317.15)),//3
        new Translation2d(inchToMeter(365.20), inchToMeter(241.64)),//4
        new Translation2d(inchToMeter(365.20), inchToMeter(75.39)),//5
        new Translation2d(inchToMeter(530.49), inchToMeter(130.17)),//6
        new Translation2d(inchToMeter(546.87), inchToMeter(158.50)),//7
        new Translation2d(inchToMeter(530.49), inchToMeter(186.83)),//8
        new Translation2d(inchToMeter(497.77), inchToMeter(186.83)),//9
        new Translation2d(inchToMeter(481.39), inchToMeter(158.50)),//10
        new Translation2d(inchToMeter(497.77), inchToMeter(130.17)),//11
        new Translation2d(inchToMeter(33.51), inchToMeter(25.80)),//12
        new Translation2d(inchToMeter(33.51), inchToMeter(291.20)),//13
        new Translation2d(inchToMeter(325.68), inchToMeter(241.64)),//14
        new Translation2d(inchToMeter(325.68), inchToMeter(75.39)),//15
        new Translation2d(inchToMeter(235.73), inchToMeter(-0.15)),//16
        new Translation2d(inchToMeter(160.39), inchToMeter(130.17)),//17
        new Translation2d(inchToMeter(144.00), inchToMeter(158.50)),//18
        new Translation2d(inchToMeter(160.39), inchToMeter(186.83)),//19
        new Translation2d(inchToMeter(193.10), inchToMeter(186.83)),//20
        new Translation2d(inchToMeter(209.49), inchToMeter(186.83)),//21
        new Translation2d(inchToMeter(193.10), inchToMeter(193.10)),//22

    };
    public static Rotation2d[] TAG_ANGLE = {null,//0
        Rotation2d.fromDegrees(126),//1
        Rotation2d.fromDegrees(234),//2
        Rotation2d.fromDegrees(270),//3
        Rotation2d.fromDegrees(0),//4
        Rotation2d.fromDegrees(0),//5
        Rotation2d.fromDegrees(300),//6
        Rotation2d.fromDegrees(0),//7
        Rotation2d.fromDegrees(60),//8
        Rotation2d.fromDegrees(120),//9
        Rotation2d.fromDegrees(180),//10
        Rotation2d.fromDegrees(240),//11
        Rotation2d.fromDegrees(54),//12
        Rotation2d.fromDegrees(306),//13
        Rotation2d.fromDegrees(180),//14
        Rotation2d.fromDegrees(180),//15
        Rotation2d.fromDegrees(90),//16
        Rotation2d.fromDegrees(240),//17
        Rotation2d.fromDegrees(180),//18
        Rotation2d.fromDegrees(120),//19
        Rotation2d.fromDegrees(120),//20
        Rotation2d.fromDegrees(0),//21
        Rotation2d.fromDegrees(300),//22
    };

    /**
     * Array of AprilTag heights from the ground.
     * Each value corresponds to either LOW, MID, or HIGH mounting position.
     * Index corresponds to AprilTag ID (0 is unused).
     */
    public static double[] TAG_HIGHT = {0,//0
        STATION_TAG_HIGHT,//1
        STATION_TAG_HIGHT,//2
        SIDE_TAG_HIGHT,//3
        BARGE_TAG_HIGHT,//4
        BARGE_TAG_HIGHT,//5
        REEF_TAG_HIGHT,//6
        REEF_TAG_HIGHT,//7
        REEF_TAG_HIGHT,//8
        REEF_TAG_HIGHT,//9
        REEF_TAG_HIGHT,//10
        REEF_TAG_HIGHT,//11
        STATION_TAG_HIGHT,//12
        STATION_TAG_HIGHT,//13
        BARGE_TAG_HIGHT,//14
        BARGE_TAG_HIGHT,//15
        SIDE_TAG_HIGHT,//16
        REEF_TAG_HIGHT,//17
        REEF_TAG_HIGHT,//18
        REEF_TAG_HIGHT,//19
        REEF_TAG_HIGHT,//20
        REEF_TAG_HIGHT,//21
        REEF_TAG_HIGHT,//22
    };

    public static final Translation2d REEF_TAG_TO_RIGHT_SCORING = new Translation2d(-0.55,-0.160);
    public static final Translation2d REEF_TAG_TO_LEFT_SCORING = new Translation2d(-0.55,0.160);
    public static final Translation2d INTAKE_TAG_TO_LEFT_SCORING = new Translation2d(-0.85,0.0);

    /**
     * Converts a measurement from inches to meters
     * @param inch Value in inches
     * @return Value in meters
     */
    public static double inchToMeter(double inch){
    return inch*0.0254;
    }
    
    // 0 is LEFT TAG(ll2), 1 is RIGHT TAG, 2 is SIDE TAG, 3 is NOTE.
    //TAG Camera mounting configuration
    public static final double[] CAM_HIGHT = {0.788,0.9,0.9,0.79};
    public static final double[] CAM_PITHC = {61.9,33,38.5,73.1};// check!!!!!!!!!!!!!
    public static final double[] CAM_YAW = {11,15.46,180+15.46,169};

    // Camera to Tag position relative to robot center
    public static final Translation2d[] ROBOT_TO_CAM = {new Translation2d(0.13, -0.27),new Translation2d(0.1043341, -0.287903),new Translation2d(0.0093341,-0.287903),new Translation2d(-0.0250,0.284)};


    // NetworkTables key for AprilTag vision data
    public static final String[] TABLE = {"limelight-bottom","limelight-front","limelight-back", "limelight-backup"};

    
    // resolution of cam
    public static final double CROP_OFSET = 0.5;





}

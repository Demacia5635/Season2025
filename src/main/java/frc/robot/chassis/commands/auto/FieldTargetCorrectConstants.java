// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldTargetCorrectConstants {
    
    public static final Translation2d avoidReefOffset = new Translation2d(2, 0);

    public static final Translation2d reefOffsetLeft = new Translation2d(0, -0.11);
    public static final Translation2d reefOffsetRight = new Translation2d(0, 0.25);
    public static Translation2d topAlgaeOffset = new Translation2d(0.54, -0.18);
    public static Translation2d bottomAlgaeOffset = new Translation2d(0.54, -0.18);

    public static Translation2d intakeOffset = new Translation2d(0.70, 0);
    public static Translation2d rightIntakeOffset = new Translation2d(0, 0.75);
    public static Translation2d leftIntakeOffset = new Translation2d(0, -0.75);

    public static final Translation2d l2Offset = new Translation2d(0.64, 0);
    public static final Translation2d l3Offset = new Translation2d(0.5, 0);
    public static final Translation2d realLeftReefOffset = new Translation2d(-0.05,-0.16);
    public static final Translation2d realRightReefOffset = new Translation2d(-0.05,0.16);

    public static Translation2d l2Left = new Translation2d(0.58, -0.14);
    public static Translation2d l2Right = new Translation2d(0.57, 0.215);
    public static Translation2d l3Left = new Translation2d(0.5, -0.14);
    public static Translation2d l3Right = new Translation2d(0.5, 0.215);
}

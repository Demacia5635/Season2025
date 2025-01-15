// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.pathPoint;
import static frc.robot.chassis.commands.auto.AutoUtils.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Auto_3Coral extends Command {
  Command cmd;
  pathPoint startingPoint = new pathPoint(5.1, 7.5, Rotation2d.fromDegrees(180), 0.0, false);
  pathPoint dumy1 = new pathPoint(4.490, 5.825, Rotation2d.fromDegrees(125), 0.0, false);
  pathPoint dumy2 = new pathPoint(4.490, 5.825, Rotation2d.fromDegrees(150), 0.0, false);
  pathPoint intakePoint = new pathPoint(1.619, 7.375, Rotation2d.fromDegrees(125), 0.0, false);
  pathPoint scoring1 = new pathPoint(5.340, 4.853, Rotation2d.fromDegrees(240), 0.0, false);
  pathPoint scoring2 = new pathPoint(4.773, 5.181, Rotation2d.fromDegrees(240), 0.0, false);
  pathPoint scoring3 = new pathPoint(4.2, 5.181, Rotation2d.fromDegrees(150), 0.0, false);
  
  public Auto_3Coral() {
  }

  @Override
  public void initialize() {
    cmd = goToMultiple(new pathPoint[]{startingPoint, scoring1, dumy1, intakePoint, dumy2, scoring2, dumy1, intakePoint, scoring3},2.0);
  }


}

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
  pathPoint point1 = new pathPoint(0.0, 0.0, Rotation2d.fromDegrees(0), 0.0, false);
  pathPoint point2 = new pathPoint(0.0, 0.0, Rotation2d.fromDegrees(0), 0.0, false);
  pathPoint point3 = new pathPoint(0.0, 0.0, Rotation2d.fromDegrees(0), 0.0, false);

  public Auto_3Coral() {
  }

  @Override
  public void initialize() {
    cmd = goToMultiple(new pathPoint[]{point1, point2, point3},2.0);
  }


}

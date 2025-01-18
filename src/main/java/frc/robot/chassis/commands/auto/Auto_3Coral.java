// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.PathPoint;
import static frc.robot.chassis.commands.auto.AutoUtils.*;
import static frc.robot.PathFollow.PathFollow.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Auto_3Coral extends Command {
  Command cmd;
  PathPoint dumyPoint = new PathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false);
  PathPoint scoring1 = new PathPoint(fieldLength - 12, fieldHeight - 2.7, Rotation2d.fromDegrees(180).minus(Rotation2d.fromDegrees(50)), 0.1, false);
  PathPoint betweenCorals = new PathPoint(13.779, 1.81, Rotation2d.fromDegrees(-9.2), 0.1, false);
  PathPoint corals = new PathPoint(16.415, 1.13, Rotation2d.fromDegrees(-56.39), 0.1, false);
  PathPoint scoring2 = new PathPoint(14.0, 2.45, Rotation2d.fromDegrees(117), 0.1, false);
  PathPoint scoring3 = new PathPoint(12.64, 2.747, Rotation2d.fromDegrees(129), 0.1, false);

  public Auto_3Coral() {
  }

  @Override
  public void initialize() {
    cmd = goToMultiple(new PathPoint[]{dumyPoint, scoring1},1.0);
    cmd.schedule();
  }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.subsystems.Chassis;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private Rotation2d wantedAngle;
  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points, Rotation2d wantedAngle) {
    this.chassis = chassis;
    this.points = points;
    this.wantedAngle = wantedAngle;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose());
  }

  @Override
  public void execute() {
    chassis.setVelocities(trajectory.calculate(chassis.getPose(), chassis.getChassisSpeeds()));

  }
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  } 

  @Override
  public boolean isFinished() {
    return trajectory.isFinishedTrajectory();
  }
}

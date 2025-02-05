// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.utils.LogManager;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private Rotation2d wantedAngle;
  private FieldTarget target;
  private boolean usePoints;
  private boolean isScoring;

  public FollowTrajectory(Chassis chassis, boolean isScoring) {
    this.chassis = chassis;
    this.usePoints = false;
    this.isScoring = isScoring;
    addRequirements(chassis);
  //  this.points.add(target.getFinishPoint());
  }

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points, Rotation2d wantedAngle) {
    this.chassis = chassis;
    this.points = points;
    this.wantedAngle = wantedAngle;
    this.usePoints = true;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    if (isScoring) {
      this.target = new FieldTarget(RobotContainer.scoringTarget.position, RobotContainer.scoringTarget.elementPosition, RobotContainer.scoringTarget.level);
    } else {
      this.target = new FieldTarget(RobotContainer.feedingTarget.position, RobotContainer.feedingTarget.elementPosition, RobotContainer.feedingTarget.level);
      new Grab(RobotContainer.gripper).schedule();
    }

    if(!usePoints){
      points = new ArrayList<PathPoint>();
      this.wantedAngle = target.getFinishPoint().getRotation();
      points.add(new PathPoint(new Translation2d(), wantedAngle));
      points.add(target.getApproachingPoint());
      points.add(target.getFinishPoint());

    }
    this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose());
    if(this.target != null) RobotContainer.arm.setState(this.target.level);
  }

  @Override
  public void execute() {
    chassis.setVelocities(trajectory.calculate(chassis.getPose(), chassis.getChassisSpeeds()));

  }
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    if (isScoring) {
      new Drop(RobotContainer.gripper).schedule();
    }
  } 

  @Override
  public boolean isFinished() {
    return trajectory.isFinishedTrajectory();
  }
}

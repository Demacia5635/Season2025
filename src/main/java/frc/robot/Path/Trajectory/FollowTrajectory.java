// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static frc.robot.vision.utils.VisionConstants.O_TO_TAG;
import static frc.robot.vision.utils.VisionConstants.TAG_ANGLE;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.RemoveAlgae;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.utils.LogManager;
import frc.robot.vision.utils.VisionConstants;

public class FollowTrajectory extends Command {
  private Chassis chassis;
  private DemaciaTrajectory trajectory;
  private ArrayList<PathPoint> points;
  private Rotation2d wantedAngle;
  private FieldTarget target;
  private boolean usePoints;
  private boolean isScoring;
  private boolean useElasticTarget;
  private Command grabCommand;
  private boolean isAlgaeRight;
  

  public FollowTrajectory(Chassis chassis, boolean isScoring) {
    this.chassis = chassis;
    this.usePoints = false;
    this.isScoring = isScoring;
    this.useElasticTarget = true;

    addRequirements(chassis);
    // this.points.add(target.getFinishPoint());
  }
  public FollowTrajectory(Chassis chassis, FieldTarget newTarget, boolean isAlgaeRight) {
    this.chassis = chassis;
    this.target = newTarget;
    this.useElasticTarget = false;
    this.usePoints = false;
    this.isAlgaeRight = isAlgaeRight;
    addRequirements(chassis);
  }

  public FollowTrajectory(Chassis chassis, FieldTarget newTarget) {
    this(chassis, newTarget, newTarget.position == POSITION.C || newTarget.position == POSITION.D || newTarget.position == POSITION.E);
  }

  public FollowTrajectory(Chassis chassis, ArrayList<PathPoint> points, Rotation2d wantedAngle) {
    this.chassis = chassis;
    this.points = points;
    this.wantedAngle = wantedAngle;
    this.usePoints = true;
    this.useElasticTarget = true;
    addRequirements(chassis);
  }

  private FieldTarget getClosestFeeder() {
    if (chassis.getPose().getTranslation().getDistance(O_TO_TAG[POSITION.FEEDER_LEFT.getId()]) > chassis.getPose().getTranslation().getDistance(O_TO_TAG[POSITION.FEEDER_RIGHT.getId()])) {
      return RobotContainer.isRed() ? FieldTarget.kFeederLeft : FieldTarget.kFeederRight;
    } else {
      return RobotContainer.isRed() ? FieldTarget.kFeederRight : FieldTarget.kFeederLeft;
    }
  }

  @Override
  public void initialize() {
    RobotContainer.robot1Strip.setAutoPath();
    if(useElasticTarget) this.target = isScoring ? RobotContainer.scoringTarget : getClosestFeeder();

    if (!usePoints) {
      points = new ArrayList<PathPoint>();
      this.wantedAngle = target.getFinishPoint(isAlgaeRight).getRotation();
      points.add(new PathPoint(new Translation2d(), wantedAngle));
      
      points.add(target.getApproachingPoint(isAlgaeRight));
      
      points.add(target.getFinishPoint(isAlgaeRight));
      if (target.elementPosition == ELEMENT_POSITION.FEEDER) {
        grabCommand = new Grab(RobotContainer.gripper).andThen(new InstantCommand(()-> RobotContainer.arm.setState(ARM_ANGLE_STATES.STARTING)));
        grabCommand.schedule();
      }

      LogManager.log(points);
    }
    this.trajectory = new DemaciaTrajectory(points, false, wantedAngle, chassis.getPose(), target.elementPosition == ELEMENT_POSITION.ALGEA);
    if (this.target != null)
      RobotContainer.arm.setState(this.target.level);
  }

  @Override
  public void execute() {
    chassis.setVelocitiesWithAccel(trajectory.calculate(chassis.getPose()), true);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {

      if (target.elementPosition == ELEMENT_POSITION.FEEDER) {
        chassis.stop();
      } 

      if (target.elementPosition == ELEMENT_POSITION.CORAL_LEFT
          || target.elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {

        chassis.stop();
        new WaitUntilCommand(RobotContainer.arm::isReady).andThen(new Drop(RobotContainer.gripper)).schedule();
      }
      if (target.elementPosition == ELEMENT_POSITION.ALGEA) {
        if (!DriverStation.isAutonomous()) {
          new RemoveAlgae(chassis, target, isAlgaeRight).schedule();
        } else {
          chassis.stop();
        }
        // LogManager.log("i do algea");
      }

    }
  }

  @Override
  public boolean isFinished() {
    return (trajectory.isFinishedTrajectory()) || 
    (!usePoints 
    && (target.position == POSITION.FEEDER_LEFT || target.position == POSITION.FEEDER_RIGHT) 
    && RobotContainer.gripper.isCoralDownSensor());
  }
}

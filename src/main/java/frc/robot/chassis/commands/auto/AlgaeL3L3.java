// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3L3 extends SequentialCommandGroup {

  public AlgaeL3L3(Chassis chassis) {
    addCommands(
      new RunCommand(()-> chassis.goTo(new Pose2d(13, 2.2, Rotation2d.fromDegrees(125)), 0.3, true), chassis)
      .until(()-> chassis.getPose().getTranslation().getDistance(new Translation2d(13, 2.2)) <= 0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), true),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), true),
      new RunCommand(()-> chassis.goTo(new Pose2d(14.24, 1.46, Rotation2d.fromDegrees(125)), 0.2, true), chassis)
      .until(()-> chassis.getPose().getTranslation().getDistance(new Translation2d(14.24, 1.46)) <= 0.2),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0 ,0)), chassis).withTimeout(0.3),
      new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(-55)), 0.3, true),
      chassis)
      .until(() -> chassis.isSeeTag(1, 1, 10)
          || chassis.getPose().getTranslation().getDistance(
              new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(-55)).getTranslation()) <= 0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
      new WaitUntilCommand(()-> RobotContainer.gripper.isCoralDownSensor()),
      new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(110)), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(6, 0, 10) || chassis.isSeeTag(6, 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    new Translation2d(14.764765315220112, 2.2)) <= 0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0 ,0)), chassis).withTimeout(0.3),
      new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(-55)), 0.3, true),
      chassis)
      .until(() -> chassis.isSeeTag(1, 1, 10)
          || chassis.getPose().getTranslation().getDistance(
              new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(-55)).getTranslation()) <= 0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
      new WaitUntilCommand(()-> RobotContainer.gripper.isCoralDownSensor()),
      new RunCommand(() -> chassis.goTo(new Pose2d(15.51, 2.45, Rotation2d.fromDegrees(160)), 0.3, true),
      chassis)
      .until(() -> chassis.isSeeTag(6, 0, 10) || chassis.isSeeTag(6, 3, 10)
          || chassis.isSeeTag(7, 0, 10) || chassis.isSeeTag(7, 3, 10)
          || chassis.getPose().getTranslation().getDistance(
              new Translation2d(15.51, 2.45)) <= 0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis)
      .withTimeout(0.4)
    );
  }
}

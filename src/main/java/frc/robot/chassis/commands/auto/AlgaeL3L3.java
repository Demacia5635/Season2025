// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.gripper.commands.AlignCoral;
import frc.robot.robot1.gripper.commands.Grab;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3L3 extends SequentialCommandGroup {

  final double FIELD_LENGTH = 17.54824934;
  final double FIELD_HEIGHT = 8.05180000;

  final boolean isRed;
  final boolean isRight;

  public AlgaeL3L3(Chassis chassis, boolean isRed, boolean isRight) {

    this.isRed = isRed;
    this.isRight = isRight;

    addCommands(
        new InstantCommand(()-> RobotContainer.arm.setState(ARM_ANGLE_STATES.ALGAE_BOTTOM))
            .alongWith(new RunCommand(() -> chassis.goTo(correctPose(3.7, 1.3, 70), 0.05, true), chassis))
            .until(()->chassis.isSeeTag(isRight ? FieldTarget.POSITION.C.getId() : POSITION.A.getId(), 0, 10) || chassis.isSeeTag(isRight ? FieldTarget.POSITION.C.getId() : POSITION.A.getId(), 3, 10)),
            
        new FollowTrajectory(chassis, new FieldTarget(isRight ? POSITION.C : POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), !isRight),
        new RemoveAlgae(chassis, new FieldTarget(isRight ? POSITION.C : POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), !isRight),
        (new RunCommand(() -> chassis.goTo(correctPose(3.821537798255507, 1.7008558315381697, 69.85764115222094), 0.3, true), chassis)
        .alongWith(new InstantCommand(()-> new AlignCoral(RobotContainer.gripper).schedule())))
        .until(()->chassis.getPose().getTranslation().getDistance(
            correctTranslation(3.821537798255507, 1.7008558315381697)) <= 0.3),
        

        new FollowTrajectory(chassis, new FieldTarget(isRight ? POSITION.C : POSITION.A, isRight ? ELEMENT_POSITION.CORAL_RIGHT : ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
        new WaitUntilCommand(() -> !RobotContainer.gripper.isCoralUpSensor()),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis).withTimeout(0.3),
        new RunCommand(
            () -> chassis
                .goTo(correctPose(3.1878867754830176, 2.4954339931530822, -120), 0.3, true),
            chassis).until(
                () -> chassis.isSeeTag(isRight ? POSITION.FEEDER_RIGHT.getId() : POSITION.FEEDER_LEFT.getId(), 1, 10)
                    || chassis.getPose().getTranslation().getDistance(
                        correctTranslation(3.1878867754830176, 2.4954339931530822)) <= 0.3),

        new FollowTrajectory(chassis, new FieldTarget(isRight ? POSITION.FEEDER_RIGHT : POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
        new WaitUntilCommand(() -> RobotContainer.gripper.isCoralDownSensor()),
        new RunCommand(
            () -> chassis
                .goTo(correctPose(2.5664175094949795, 2.094113517186559, 43.97075500034001), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(isRight ? POSITION.C.getId() : POSITION.A.getId(), 0, 10) || chassis.isSeeTag(isRight ? POSITION.C.getId() : POSITION.A.getId(), 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    correctTranslation(2.783484024779888e+0, 2.2)) <= 0.3),

        new FollowTrajectory(chassis, new FieldTarget(isRight ? POSITION.C : POSITION.A, isRight ? ELEMENT_POSITION.CORAL_LEFT : ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3)),
        new WaitUntilCommand(() -> !RobotContainer.gripper.isCoralUpSensor()),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis).withTimeout(0.3),
        new RunCommand(
            () -> chassis
                .goTo(correctPose(2.783484024779888e+0, 2.2, -55), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(isRight ? POSITION.FEEDER_RIGHT.getId() : POSITION.FEEDER_LEFT.getId(), 1, 10)
                || chassis.getPose().getTranslation().getDistance(
                    correctTranslation(2.783484024779888e+0, 2.2)) <= 0.3),

        new FollowTrajectory(chassis, new FieldTarget(isRight ? POSITION.FEEDER_RIGHT : POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
        new WaitUntilCommand(() -> RobotContainer.gripper.isCoralDownSensor()),
        new RunCommand(
            () -> chassis
                .goTo(
                    correctPose(2.066323076202146e+0, 1.7987086802306866, 160),
                    0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(isRight ? POSITION.C.getId() : POSITION.A.getId(), 0, 10) || chassis.isSeeTag(isRight ? POSITION.C.getId() : POSITION.A.getId(), 3, 10)
                || chassis.isSeeTag(POSITION.B.getId(), 0, 10) || chassis.isSeeTag(POSITION.B.getId(), 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    correctTranslation(2.066323076202146e+0, 1.7987086802306866)) <= 0.3
                    && Math.abs(
                        chassis.getPose().getRotation().getDegrees() - (isRight ? isRed ? -160 : 160 : isRed ? 160 : -160)) <= 5),

        new FollowTrajectory(chassis, new FieldTarget(POSITION.B, isRight ? ELEMENT_POSITION.CORAL_RIGHT : ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis)
            .withTimeout(0.4));
  }

  private Pose2d correctPose(double x, double y, double angle) {
    return new Pose2d(
      isRed ? FIELD_LENGTH - x : x,
      isRight ? isRed ? FIELD_HEIGHT - y : y : isRed ? y : FIELD_HEIGHT - y,
      Rotation2d.fromDegrees(
        isRight ? isRed ? -angle : angle : isRed ? angle : -angle));
  }

  private Translation2d correctTranslation(double x, double y) {
    return new Translation2d(
      isRed ? FIELD_LENGTH - x : x,
      isRight ? isRed ? FIELD_HEIGHT - y : y : isRed ? y : FIELD_HEIGHT - y);
  }
}

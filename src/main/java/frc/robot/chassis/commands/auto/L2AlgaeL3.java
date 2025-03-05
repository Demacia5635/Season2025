// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.robot1.gripper.commands.Grab;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2AlgaeL3 extends SequentialCommandGroup {

  public L2AlgaeL3(Chassis chassis) {

    addCommands(
        new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L2)),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis).withTimeout(0.3),
        new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER)),
        new WaitUntilCommand(() -> RobotContainer.gripper.isCoralDownSensor()),
        new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(125)), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(6, 0, 10) || chassis.isSeeTag(6, 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    new Pose2d(14.764765315220112, 2.2, Rotation2d.fromDegrees(125)).getTranslation()) <= 0.3),
        new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
        new RemoveAlgae(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
        new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3)),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis).withTimeout(0.3),
        new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER)),
        new WaitUntilCommand(() -> RobotContainer.gripper.isCoralDownSensor()));
  }
}

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3L3 extends SequentialCommandGroup {

  public AlgaeL3L3(Chassis chassis) {
    addCommands(
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), false),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), false),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0 ,0)), chassis),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0 ,1)), chassis).withTimeout(0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
      new WaitUntilCommand(()-> RobotContainer.gripper.isCoralDownSensor()),
      new RunCommand(()->chassis.goTo(new Pose2d(14.764765315220112, 2.2286484904026063, Rotation2d.fromDegrees(125)), 0.3, false), chassis)
      .until(()->chassis.isSeeTag(6, 0, 5) || chassis.isSeeTag(6, 3, 5)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 1)), chassis),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER))
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import java.lang.System.Logger.Level;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2AlgaeL3 extends SequentialCommandGroup {
  Chassis chassis;
  private ArrayList<PathPoint> feederToA = new ArrayList<PathPoint>();
  
  
  
  
  public L2AlgaeL3(Chassis chassis, boolean L2Right) {
    feederToA.add(new PathPoint(new Translation2d(), new Rotation2d()));
    feederToA.add(new PathPoint(new Translation2d(14.5, 2), Rotation2d.fromDegrees(125)));





    addCommands(
      // new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L2)),
      // new RunCommand(()->chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 1.5,0)), chassis).withTimeout(0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), false),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM), false),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, -1, 0)), chassis).withTimeout(0.1),
      new RunCommand(()-> chassis.stop(), chassis).withTimeout(0.3),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
      new RunCommand(()->chassis.goTo(new Pose2d(15.38, 1.75, Rotation2d.fromDegrees(-40)), 0.3, false), chassis)
      .until(()->chassis.isSeeTag(1, 1, 5)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
      new WaitUntilCommand(()->RobotContainer.gripper.isCoralDownSensor()),
      new RunCommand(()->chassis.goTo(new Pose2d(14.764765315220112, 2.2286484904026063, Rotation2d.fromDegrees(125)), 0.3, false), chassis)
      .until(()->chassis.isSeeTag(6, 0, 5) || chassis.isSeeTag(6, 3, 5)),
      new WaitCommand(0.2),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3)),
      new RunCommand(()->chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0,0)), chassis).withTimeout(0.3),
      new RunCommand(()->chassis.goTo(new Pose2d(15.6, 2, Rotation2d.fromDegrees(-50)), 0.3, false), chassis)
      .until(()->chassis.isSeeTag(1, 1, 5)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER)),
      new WaitUntilCommand(()->RobotContainer.gripper.isCoralDownSensor()),
      new RunCommand(()->chassis.goTo(new Pose2d(14.764765315220112, 2.2286484904026063, Rotation2d.fromDegrees(125)), 0.3, false), chassis)
      .until(()->chassis.isSeeTag(6, 0, 5) || chassis.isSeeTag(6, 3,  5)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER))
     );
  }
}

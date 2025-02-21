package frc.robot.chassis.commands.auto;

import java.lang.reflect.Field;
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
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.AlignCoral;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class Test extends SequentialCommandGroup {
  Chassis chassis;
  Arm arm;
  Gripper gripper;

  public Test() {
    PathPoint dummyPoint = new PathPoint(Translation2d.kZero, Rotation2d.kZero);
    FieldTarget aAlgaeBottom = new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM);
    FieldTarget feederLeft = new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER);
    FieldTarget feederRight = new FieldTarget(POSITION.FEEDER_RIGHT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER);
    FieldTarget coralLeft = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

    FieldTarget coralRight = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3);

    this.chassis = RobotContainer.chassis;
    this.arm = RobotContainer.arm;
    this.gripper = RobotContainer.gripper;

    Command goToFeederLeft = new FollowTrajectory(chassis, feederLeft);
    Command goToFeederRight = new FollowTrajectory(chassis, feederRight);

    addCommands(
        new FollowTrajectory(chassis, aAlgaeBottom, false).andThen(new RemoveAlgae(chassis, aAlgaeBottom, false)
            .alongWith(new InstantCommand(() -> new AlignCoral(gripper).schedule()))),
        new FollowTrajectory(chassis, new ArrayList<PathPoint>() {
          {
            add(dummyPoint);
            add(new PathPoint(new Translation2d(3.3, 6), Rotation2d.fromDegrees(-60)));
          }
        }, Rotation2d.fromDegrees(-60)).until(() -> chassis.isSeeTag(coralRight.position.getId(), 0, 2)
            || chassis.isSeeTag(coralRight.position.getId(), 3, 2)),
        new FollowTrajectory(chassis, coralRight),
        new WaitUntilCommand(() -> !gripper.isCoralDownSensor()).withTimeout(0.2),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 4)), chassis).withTimeout(0.2),
        new FollowTrajectory(chassis, new ArrayList<PathPoint>() {
          {
            add(dummyPoint);
            add(feederLeft.getApproachingPoint());
          }
        }, Rotation2d.fromDegrees(130)).until(() -> chassis.isSeeTag(feederLeft.position.getId(), 1, 10)),
        new FollowTrajectory(chassis, feederLeft),
        new WaitUntilCommand(() -> gripper.isCoralDownSensor()),
        new FollowTrajectory(chassis, new ArrayList<PathPoint>() {
          {
            add(dummyPoint);
            add(new PathPoint(new Translation2d(3.3, 6), Rotation2d.fromDegrees(-60)));
          }
        }, Rotation2d.fromDegrees(-60)).until(() -> chassis.isSeeTag(coralLeft.position.getId(), 0, 2.5)
            || chassis.isSeeTag(coralLeft.position.getId(), 3, 2.5)),
        new FollowTrajectory(chassis, coralLeft));
  }

}

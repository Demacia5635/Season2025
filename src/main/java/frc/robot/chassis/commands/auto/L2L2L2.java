package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;

public class L2L2L2 extends SequentialCommandGroup {
    
    public L2L2L2(Chassis chassis) {

        addCommands(
            new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L2)),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis).withTimeout(0.3),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER)),
            new WaitUntilCommand(()-> RobotContainer.gripper.isCoralDownSensor()),
            new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 3, Rotation2d.fromDegrees(125)), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(7, 0, 10) || chassis.isSeeTag(7, 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    new Pose2d(14.764765315220112, 3, Rotation2d.fromDegrees(125)).getTranslation()) <= 0.3),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L2)),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0.5, 0)), chassis).withTimeout(0.3),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER)),
            new WaitUntilCommand(()-> RobotContainer.gripper.isCoralDownSensor()),
            new RunCommand(() -> chassis.goTo(new Pose2d(14.764765315220112, 3, Rotation2d.fromDegrees(125)), 0.3, true),
            chassis)
            .until(() -> chassis.isSeeTag(7, 0, 10) || chassis.isSeeTag(7, 3, 10)
                || chassis.getPose().getTranslation().getDistance(
                    new Pose2d(14.764765315220112, 3, Rotation2d.fromDegrees(125)).getTranslation()) <= 0.3),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0.5, 0)), chassis).withTimeout(0.3),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER))
        );
    }
}

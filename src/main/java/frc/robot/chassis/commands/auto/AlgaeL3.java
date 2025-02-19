package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;

public class AlgaeL3 extends SequentialCommandGroup {
    
    public AlgaeL3(Chassis chassis) {

        addCommands(
            new FollowTrajectory(chassis, new FieldTarget(POSITION.E, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
            new RunCommand(()-> chassis.goTo(new Pose2d(10.462589603882295, 3.9189009240081867, Rotation2d.kZero), 0.3, true), chassis),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.E, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis)
        );
    }
}

package frc.robot.chassis.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;

public class AlgaeL3 extends SequentialCommandGroup {
    
    public AlgaeL3(Chassis chassis) {

        PathPoint dummyPoint = new PathPoint(Translation2d.kZero, Rotation2d.kZero);
        FieldTarget eAlgae = new FieldTarget(POSITION.E, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM);
        FieldTarget eCoral = new FieldTarget(POSITION.E, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

        addCommands(
            new FollowTrajectory(chassis, new FieldTarget(POSITION.E, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
            new FollowTrajectory(chassis, new ArrayList<PathPoint>() {
                {
                    add(dummyPoint);
                    add(eCoral.getApproachingPoint());
                }
            }, Rotation2d.kPi),
            new FollowTrajectory(chassis, new FieldTarget(POSITION.E, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3)),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis)
        );
    }
}

package frc.robot.chassis.commands.auto;

import java.util.ArrayList;

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
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.AlignCoral;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class AlgaeL3 extends SequentialCommandGroup {
    
    public AlgaeL3(Chassis chassis, Arm arm, Gripper gripper) {

        PathPoint dummyPoint = new PathPoint(Translation2d.kZero, Rotation2d.kZero);
        FieldTarget eAlgae = new FieldTarget(POSITION.E, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM);
        FieldTarget eCoral = new FieldTarget(POSITION.E, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

        addCommands(
            new FollowTrajectory(chassis, eAlgae),
            new RemoveAlgae(chassis, eCoral)
            .alongWith(new InstantCommand(()-> new AlignCoral(gripper).schedule())),
            new WaitCommand(0.1).alongWith(new InstantCommand(() -> arm.setState(LEVEL.L3))),
            new FollowTrajectory(chassis, eCoral),
            new WaitUntilCommand(() -> !gripper.isCoralUpSensor()).alongWith(new InstantCommand(() -> new Drop(gripper).schedule())),
            new WaitUntilCommand(() -> gripper.isCoralUpSensor()),
            new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis).withTimeout(0.3)
        );
    }
}

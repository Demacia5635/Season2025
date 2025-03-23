// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3L3 extends SequentialCommandGroup {

    final double FIELD_LENGTH = 17.54824934;
    final double FIELD_HEIGHT = 8.05180000;

    boolean isRed;
    boolean isRight;

  public AlgaeL3L3(Chassis chassis, Arm arm, Gripper gripper, boolean isRed, boolean isRight) {

    this.isRed = isRed;
    this.isRight = isRight;



    PathPoint dummyPoint = PathPoint.kZero;
    // PathPoint infrontReef = new PathPoint(correctTranslation(3.3, FIELD_HEIGHT - 6), correctRotation(-120));
    // FieldTarget fAlgaeTarget = new FieldTarget(isRight ? POSITION.D : POSITION.F, ELEMENT_POSITION.ALGEA,
    //     LEVEL.ALGAE_TOP);
    FieldTarget aAlgaePoint = new FieldTarget(isRight ? POSITION.C : POSITION.A, ELEMENT_POSITION.ALGEA,
        LEVEL.ALGAE_BOTTOM);
    FieldTarget fAlgaePoint = new FieldTarget(isRight ? POSITION.D : POSITION.F, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_TOP);
    FieldTarget feeder = isRight
        ? new FieldTarget(POSITION.FEEDER_RIGHT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER)
        : new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER);

    FieldTarget coralF = new FieldTarget(isRight ? POSITION.D : POSITION.F,
        isRight ? ELEMENT_POSITION.CORAL_LEFT : ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L2);
    FieldTarget coralLeft = new FieldTarget(isRight ? POSITION.C : POSITION.A,
        isRight ? ELEMENT_POSITION.CORAL_RIGHT : ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);
    FieldTarget coralRight = new FieldTarget(isRight ? POSITION.C : POSITION.A,
        isRight ? ELEMENT_POSITION.CORAL_LEFT : ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L3);
    FieldTarget backupCoral = new FieldTarget(isRight ? POSITION.C : POSITION.A,
        isRight ? ELEMENT_POSITION.CORAL_RIGHT : ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2);

    addCommands(
        // new FollowTrajectory(chassis, coralF),
        // (new WaitUntilCommand(() -> !gripper.isCoral())
        //     .alongWith(new InstantCommand(() -> new Drop(gripper).schedule())))
        //     .withTimeout(0.7),
        // // new FollowTrajectory(chassis, fAlgaePoint),
        // AutoUtils.removeAlgae(true),
        // new WaitCommand(0.1),
        

        // new FollowTrajectory(chassis, feeder),
        // new WaitUntilCommand(() -> gripper.isCoral()),
        // new WaitCommand(0.1),

        
        // (new FollowTrajectory(chassis, new ArrayList<PathPoint>() {
        //     {
        //         add(dummyPoint);
                
        //         add(new PathPoint(correctPose(FIELD_LENGTH - 15.063582653364106, FIELD_HEIGHT - 1.841681426027737, 125)));
        //     }
        // }, correctRotation(125))).until(()->chassis.isSeeTag(0) || chassis.isSeeTag(3)),
        
        new FollowTrajectory(chassis, aAlgaePoint)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.PRE_ALGAE_BOTTOM))),
        AutoUtils.removeAlgae(false),
        new WaitCommand(0.1),
        
        new FollowTrajectory(chassis, coralRight)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.L3))),
        (new WaitUntilCommand(() -> !gripper.isCoral())
            .alongWith(new InstantCommand(() -> new Drop(gripper).schedule())))
            .withTimeout(0.7),
        new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis)
            .withTimeout(0.2),
        
            
        new FollowTrajectory(chassis, feeder)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.CORAL_STATION))),
        new WaitUntilCommand(() -> gripper.isCoral()),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis)
            .withTimeout(0.1)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.L3))),
        new InstantCommand(()-> chassis.stop(), chassis),

        new FollowTrajectory(chassis, coralLeft)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.L3))),
        !(gripper.getCurrentCommand() instanceof Drop) ? new InstantCommand(() -> new Drop(gripper).schedule())
            : new InstantCommand(),
        new WaitUntilCommand(() -> !gripper.isCoralUpSensor())
            .alongWith(new InstantCommand(() -> new Drop(gripper).schedule())),
        new WaitCommand(0.1),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis).withTimeout(0.2),

        new FollowTrajectory(chassis, feeder)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.CORAL_STATION))),
        new WaitUntilCommand(() -> gripper.isCoral()),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, 0)), chassis)
            .withTimeout(0.1)
            .raceWith(new RunCommand(() -> arm.setState(ARM_ANGLE_STATES.L2))),

        new FollowTrajectory(chassis, backupCoral),
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-2, 0, 0)), chassis)
            .withTimeout(0.2),
            
        new FollowTrajectory(chassis, feeder),
        new WaitUntilCommand(()-> gripper.isCoral()),
        new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-1, 0, 0)), chassis)
    );
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

    private Rotation2d correctRotation(double angle) {
        return Rotation2d.fromDegrees(
                isRight ? isRed ? -angle : angle : isRed ? angle : -angle);
    }
}

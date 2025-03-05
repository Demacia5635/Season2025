// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2AlgaeAlgaeAlgae extends SequentialCommandGroup {

  public L2AlgaeAlgaeAlgae(Chassis chassis) {
    addCommands(
      new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0 ,0)), chassis).withTimeout(0.35),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_TOP)),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_TOP)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0 ,0)), chassis).withTimeout(0.35),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.A, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_BOTTOM)),
      new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0 ,0)), chassis).withTimeout(0.35),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_TOP)),
      new RemoveAlgae(chassis, new FieldTarget(POSITION.B, ELEMENT_POSITION.ALGEA, LEVEL.ALGAE_TOP)),
      new FollowTrajectory(chassis, new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER))
    );
  }
}

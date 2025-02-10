// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.subsystems.Gripper;

import static frc.robot.vision.utils.VisionConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullAlign extends Command {
  private Chassis chassis;
  private Arm arm;
  private Gripper gripper;
  private FieldTarget target;

  private Translation2d reelElement;
  private Translation2d robotPose;
  private Translation2d robotToTarget;
  private Translation2d robotToReelTarget;

  private double maxVel = 1;

  public static final Translation2d reelLeftReefOffset = new Translation2d(-0.05,-0.16);
  public static final Translation2d reelRightReefOffset = new Translation2d(-0.05,0.16);

  public FullAlign(Chassis chassis, Arm arm, Gripper gripper, FieldTarget target) {
    this.chassis = chassis;
    this.arm = arm;
    this.gripper = gripper;
    this.target = target;
  }

  @Override
  public void initialize() {
    reelElement = getElement(target.position.getId(), target.elementPosition == ELEMENT_POSITION.CORAL_RIGHT ? reelRightReefOffset : reelLeftReefOffset).getTranslation();

  }

  @Override
  public void execute() {
    robotPose = chassis.getPose().getTranslation();
    robotToTarget = target.getFinishPoint().getTranslation().minus(robotPose);
    robotToReelTarget = reelElement.minus(robotPose);

    chassis.setVelocitiesRotateToAngleOld(new ChassisSpeeds(Math.min(robotToTarget.getX()* 0.5, maxVel), Math.min(robotToTarget.getY()*0.5, maxVel), 0), robotToReelTarget.getAngle());
    //arm.setState(ARM_ANGLE_STATES.)
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public PathPoint getElement(int elementTag, Translation2d offset){
    Translation2d originToTag = O_TO_TAG[elementTag];
    offset = offset.rotateBy(TAG_ANGLE[elementTag]);
    return new PathPoint(originToTag.plus(offset), TAG_ANGLE[elementTag].plus(Rotation2d.fromDegrees(180)), 0.2);
  }
}

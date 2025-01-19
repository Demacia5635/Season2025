// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.chassis.commands.auto;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

import static frc.robot.vision.utils.VisionConstants.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private Chassis chassis;
  private boolean isRight;
  private Translation2d robotToTag;
  private Translation2d tagToTarget;
  private Rotation2d targetAngle;
  private Translation2d robotToTarget;
  private double maxVel = 3.8;


  public AlignToTag(Chassis chassis, boolean isRight) {
    this.chassis = chassis;
    this.isRight = isRight;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (chassis.tag != null && chassis.tag.robotToTagFC != null && chassis.tag.cameraID != null && chassis.tag.tagID != 0){
      robotToTag = chassis.tag.robotToTagFC;
      tagToTarget = isRight ? REEF_TAG_TO_RIGHT_SCORING : REEF_TAG_TO_LEFT_SCORING;
      targetAngle = TAG_ANGLE[(int)chassis.tag.tagID];
      tagToTarget = tagToTarget.rotateBy(targetAngle);
      robotToTarget = robotToTag.plus(tagToTarget);
      chassis.setVelocitiesRotateToAngle(new ChassisSpeeds(Math.min(robotToTarget.getX()*6, maxVel), Math.min(robotToTarget.getY()*6, maxVel), 0), targetAngle);

    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setVelocities(new ChassisSpeeds(0,0,0));

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.tag.cameraID == null || Math.abs(robotToTarget.getNorm()) < 0.01;
  }
}




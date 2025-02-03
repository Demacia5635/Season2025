// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.chassis.commands.auto;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private Chassis chassis;
  private boolean isRight;
  private boolean isReef;
  private Translation2d robotToTag;
  private Translation2d tagToTarget;
  private Rotation2d targetAngle;
  private Translation2d robotToTarget = new Translation2d(10,10);
  private double maxVel = 2;
  private Integer tagID = null;
  private Timer timer;
  private boolean isAuto;

  public AlignToTag(Chassis chassis, boolean isRight, boolean isReef, boolean isAuto) {
    this.chassis = chassis;
    this.isRight = isRight;
    this.isReef = isReef;
    this.isAuto = isAuto;
  }
  @Override
  public void initialize() {
    // if (chassis.tag.tagID != 0){
    //   tagID = (int)chassis.tag.tagID;
    // }
    // if(!isReef) maxVel = 2;
    // timer = new Timer();
    // timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if (chassis.tag.tagID != 0){
    //   tagID = (int)chassis.tag.tagID;
    // }
    // if(tagID != null){
    //   robotToTag = O_TO_TAG[tagID].minus(chassis.getPose().getTranslation());
    //   tagToTarget = isReef ? (isRight ? REEF_TAG_TO_RIGHT_SCORING : REEF_TAG_TO_LEFT_SCORING) : INTAKE_TAG_TO_LEFT_SCORING;
    //   targetAngle = TAG_ANGLE[tagID].minus(Rotation2d.fromDegrees(180));
    //   tagToTarget = tagToTarget.rotateBy(targetAngle);
    //   robotToTarget = robotToTag.plus(tagToTarget);
    //   chassis.setVelocitiesRotateToAngle(new ChassisSpeeds(Math.min(robotToTarget.getX()* (isAuto ? 4 : 2), maxVel), Math.min(robotToTarget.getY()*(isAuto ? 4 : 2), maxVel), 0), targetAngle);
    // }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setVelocities(new ChassisSpeeds(0,0,0));

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tagID == null || Math.abs(robotToTarget.getNorm()) < 0.02;
  }
}




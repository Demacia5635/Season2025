// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.subsystems.Chassis;


import static frc.robot.chassis.commands.auto.AutoUtils.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private Chassis chassis;
  private Rotation2d targetAngle;
  private Translation2d robotToTarget;
  private double maxVel = 0.7;
  private FIELD_POSITION position;
  private ELEMENT element;
  private LEVEL level;
  private Translation2d offset;
  private Timer timer = new Timer();
  private double cycleCount = 0;
  double kP = 3;
  

  public AutoAlign(Chassis chassis, FIELD_POSITION position, ELEMENT element, LEVEL level) {
    this.chassis = chassis;
    this.position = position;
    this.element = element;
    this.level = level;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (fieldElements == null || chassis == null) {
      return;
    }

    targetAngle = fieldElements.get(position).getRotation();
    offset = new Translation2d(
      (level == LEVEL.FEEDER) ? FEEDER_DIST : ((level == LEVEL.L2) ? L2DIST : L3DIST),
      (element == ELEMENT.FEEDER || element == ELEMENT.ALGAE) ? 0 : (element == ELEMENT.CORAL_LEFT) ? SIDE_DIST : SIDE_DIST
    );
    offset = offset.rotateBy(targetAngle.rotateBy(Rotation2d.fromDegrees(180)));
    robotToTarget = fieldElements.get(position).getTranslation().minus(chassis.getPose().getTranslation()).plus(offset);
    
    double vel = robotToTarget.getNorm() * kP;

    chassis.setVelocitiesRotateToAngle(new ChassisSpeeds(
      Math.min(vel * robotToTarget.getAngle().getCos(), maxVel),
      Math.min(vel * robotToTarget.getAngle().getSin(), maxVel),
      0
      ), targetAngle);
    
    
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setVelocities(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    
    return element == ELEMENT.FEEDER ? RobotContainer.gripper.isCoral() : Math.abs(robotToTarget.getNorm()) < 0.02;
  }
}



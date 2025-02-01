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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.AutoUtils.ELEMENT;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.commands.auto.AutoUtils.LEVEL;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.utils.LogManager;

import static frc.robot.chassis.commands.auto.AutoUtils.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private Chassis chassis;
  private Arm arm;
  private Gripper gripper;
  private Rotation2d targetAngle;
  private Translation2d robotToTarget;
  private Translation2d target;
  private double maxVel = 1.5;
  private FIELD_POSITION position;
  private ELEMENT element;
  private LEVEL level;
  private Translation2d offset;
  private Timer timer = new Timer();
  double kP = 2.5;
  

  public AutoAlign(Chassis chassis, Arm arm, Gripper gripper, FIELD_POSITION position, ELEMENT element, LEVEL level) {
    this.chassis = chassis;
    this.arm = arm;
    this.gripper = gripper;
    this.position = position;
    this.element = element;
    this.level = level;
  }

  @Override
  public void initialize() {
    timer.start();
    if(level == LEVEL.FEEDER){
      new Grab(gripper).alongWith(new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.CORAL_STATION)));
    }
    else if(level == LEVEL.L2_RIGHT || level == LEVEL.L2_LEFT){
      new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.L2_TOUCHING));
    }
    else if(level == LEVEL.L3_RIGHT || level == LEVEL.L3_LEFT){
      new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.L3_TOUCHING));
    }
    else if(level == LEVEL.ALGAE_BOTTOM){
      new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.ALGAE_OVER));
    }
    else if(level == LEVEL.ALGAE_TOP){
      new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.ALGAE_UNDER));
    }
  }

  @Override
  public void execute() {
    if (fieldElements == null || chassis == null) {
      return;
    }

    targetAngle = fieldElements.get(position).getRotation();
    offset = new Translation2d(
      (level == LEVEL.FEEDER) ? FEEDER_DIST : ((level == LEVEL.L2_RIGHT || level == LEVEL.L2_LEFT) ? L2DIST : L3DIST),
      (element == ELEMENT.FEEDER || element == ELEMENT.ALGAE) ? 0 : (element == ELEMENT.CORAL_LEFT) ? LEFT_SIDE_DIST : RIGHT_SIDE_DIST
    );
    offset = offset.rotateBy(targetAngle.rotateBy(Rotation2d.fromDegrees(180)));
    target = fieldElements.get(position).getTranslation().plus(offset);
    robotToTarget = target.minus(chassis.getPose().getTranslation());
    
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
    if(level == LEVEL.L2_RIGHT || level == LEVEL.L2_LEFT || level == LEVEL.L3_RIGHT || level == LEVEL.L3_LEFT){
      new Drop(gripper);
    }
  }

  @Override
  public boolean isFinished() {
    
    return element == ELEMENT.FEEDER ? RobotContainer.gripper.isCoral() : Math.abs(robotToTarget.getNorm()) < 0.02;
  }
}



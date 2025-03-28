// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReef extends Command {
  Chassis chassis;
  Translation2d controlPoint;
  Rotation2d finishVelocityHeading;
  double maxVel = 3;
  double maxAccel = 6;
  public GoToReef(Chassis chassis, Translation2d reefPoint, Rotation2d finishVelocityHeading) {
    this.chassis = chassis;
    this.finishVelocityHeading = finishVelocityHeading;
    this.controlPoint = reefPoint.plus(new Translation2d(0.5, finishVelocityHeading.rotateBy(Rotation2d.k180deg)));
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d diffVector = controlPoint.minus(chassis.getPose().getTranslation()).rotateBy(Rotation2d.k180deg);
    
    Rotation2d wantedVelocityHeading = (finishVelocityHeading.plus(diffVector.getAngle())).div(2);
    double wantedVelocity = Math.min(maxVel, getVelocity(diffVector.getNorm())); 
    chassis.setVelocities(new Translation2d(wantedVelocity, wantedVelocityHeading), 0);
  }
  
  private double getVelocity(double distance){
    return Math.min(maxVel, Math.sqrt(2 * maxAccel * distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getPose().getTranslation().getDistance(controlPoint) < 0.03;
  }
}

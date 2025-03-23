// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.Utils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToFinish extends Command {
  private ProfiledPIDController drivePID = new ProfiledPIDController(1.2, 0, 0, new Constraints(1.5, 2));
  private ProfiledPIDController rotationPID = new ProfiledPIDController(1.2, 0, 0, new Constraints(1.5, 2));
  Pose2d target;
  Chassis chassis;
  public goToFinish(Pose2d target, Chassis chassis) {
    this.target = target;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d vectorError = target.getTranslation().minus(chassis.getPose().getTranslation());
    double rotationError = target.getRotation().minus(chassis.getPose().getRotation()).getRadians();
    double vX = drivePID.calculate(-vectorError.getX(), 0);
    double vY = drivePID.calculate(-vectorError.getY(), 0);
    double omega = rotationPID.calculate(-rotationError, 0);
    chassis.setVelocitiesWithAccel(new ChassisSpeeds(vX, vY, omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return target.minus(chassis.getPose()).getTranslation().getNorm() <= 0.03;
  }
}

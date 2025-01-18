// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import frc.robot.chassis.PathFollow.PathFollow;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.PathFollow.Util.pathPoint;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.KeepSpinning;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.chassis.subsystems.SwerveModule;
import frc.robot.utils.LogManager;


public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  LogManager logManager;
  public static Boolean isRed = false;
  public Chassis chassis;
  Drive drive;
  double num = 0;
  private System sysid;


  public RobotContainer() {
    robotContainer = this;
    logManager = new LogManager();
    chassis = new Chassis();
    drive = new Drive(chassis, new CommandXboxController(0));
    chassis.setDefaultCommand(drive);
    SmartDashboard.putData("RC", this);
    configureBindings();
  }
  public double getNum(){ return num;}
  public void setNum(double num){this.num = num;}


  private void configureBindings() {

  }

  public static void isRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("NUM", ()->getNum(), (double num)->setNum(num));
    builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::isRed);
  }

  public Command getAutonomousCommand() {

    return new RunCommand(()->chassis.setVelocities(new ChassisSpeeds(1, 0, 0)), chassis);
    /*chassis.resetPose();
    return new PathFollow(new pathPoint[]{
      new pathPoint(new Translation2d(), new Rotation2d()), new pathPoint(new Translation2d(2, 0), new Rotation2d()), new pathPoint(new Translation2d(2, 2), new Rotation2d(), 0.3)
    }, Rotation2d.fromDegrees(90));
    //return new Auto_3Coral();*/
  }
}

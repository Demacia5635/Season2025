// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.KeepSpinning;
import frc.robot.chassis.commands.auto.AlignToTag;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.Auto_3Coral;
import frc.robot.chassis.commands.auto.goToPlace;
import frc.robot.chassis.commands.auto.AutoUtils.ELEMENT;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
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
  private CommandXboxController controller;


  public RobotContainer() {
    robotContainer = this;
    logManager = new LogManager();
    chassis = new Chassis();
    drive = new Drive(chassis, new CommandXboxController(0));

    
    AutoUtils a = new AutoUtils(); //no need to use only to update values

    chassis.setDefaultCommand(drive);
    SmartDashboard.putData("RC", this);
    controller = new CommandXboxController(0);
    configureBindings();
  }
  public double getNum(){ return num;}
  public void setNum(double num){this.num = num;}


  private void configureBindings() {
    controller.y().onTrue(new goToPlace(FIELD_POSITION.FEEDER_LEFT, ELEMENT.FEEDER, 3.5));
    controller.x().onTrue(new InstantCommand(()->chassis.setGyroAngle(chassis.tag.alignRobot())));

    controller.leftBumper().onTrue(new goToPlace(FIELD_POSITION.A, ELEMENT.CORAL_LEFT, 3.5));
    controller.rightBumper().onTrue(new goToPlace(FIELD_POSITION.A, ELEMENT.CORAL_RIGHT, 3.5));

    controller.a().onTrue(new goToPlace(FIELD_POSITION.B, ELEMENT.CORAL_LEFT, 3.5));
    controller.b().onTrue(new goToPlace(FIELD_POSITION.B, ELEMENT.CORAL_RIGHT, 3.5));



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
    return new Auto_3Coral();  
  }
}

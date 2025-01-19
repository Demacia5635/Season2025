// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.proto.System;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.KeepSpinning;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.chassis.subsystems.SwerveModule;
import frc.robot.utils.LogManager;


public class RobotContainer implements Sendable{
  
  LogManager logManager;
  public static Boolean isRed = false;
  Chassis chassis;
  Drive drive;
  double num = 0;
  private System sysid;


  public RobotContainer() {
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

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("NUM", ()->getNum(), (double num)->setNum(num));
  }

  public Command getAutonomousCommand() {
    return new KeepSpinning(chassis);
  }
}

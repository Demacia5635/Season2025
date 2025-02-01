// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.proto.System; // Removed to avoid conflict with java.lang.System
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PathFollow.PathFollow;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.PathFollow.Util.StationNav;
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
import static frc.robot.chassis.commands.auto.AutoUtils.*;



public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  LogManager logManager;
  public static Boolean isRed = false;
  public int id = 0;
  public int elementid = 0;
  public Chassis chassis;
  Drive drive;
  double num = 0;
  private System sysid;
  private CommandXboxController controller;
  PathFollow follow;


  public RobotContainer() {
    robotContainer = this;
    logManager = new LogManager();
    chassis = new Chassis();
    drive = new Drive(chassis, new CommandXboxController(0));

    
    AutoUtils a = new AutoUtils(); //no need to use only to update values

    chassis.setDefaultCommand(drive);
    SmartDashboard.putData("RC", this);
    controller = new CommandXboxController(0);

    this.follow = new PathFollow(StationNav.genLineByDis(new Translation2d(6,6), new Pose2d(0,0, new Rotation2d(0)), 2),2,true,true);

    this.follow.initialize();

    PathPoint[] points = StationNav.genLineByDis(new Translation2d(6,6), new Pose2d(0,0, new Rotation2d(0)), 2);
    double[] radius = this.follow.getRadius();

    System.out.println("Points : ");
    for(int i = 0; i < points.length; i++)
    {
      System.out.println(points[i]);
    }


    System.out.println("Radius : ");
    for(int i = 0; i < radius.length; i++)
    {
      System.out.println(i + " : " + radius[i]);
    }



    configureBindings();
  }
  public double getNum(){ return num;}
  public void setNum(double num){this.num = num;}


  private void configureBindings() {
    Trigger controllerOverride = new Trigger(
      ()->new Translation2d(controller.getLeftX(), controller.getLeftY()).getNorm() <= 0.1);

    controllerOverride.onTrue(drive);
    controller.y().onTrue(new goToPlace(FIELD_POSITION.D, ELEMENT.ALGAE, 3.5));
    controller.x().onTrue(new InstantCommand(()->chassis.setGyroAngle(chassis.tag.alignRobot())));


    controller.start().onTrue(new AlignToTag(chassis, false, true, false));



  }

  public static void isRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }
  public void setID(int id) {
    this.id = id;
  }
  public int getID() {
    return this.id;
  }
  public void setElementID(int id) {
    this.elementid = id;
  }
  public int getElementID() {
    return this.elementid;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("NUM", ()->getNum(), (double num)->setNum(num));
    builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::isRed);
    builder.addDoubleProperty("ID", this::getID, (double id)->setID((int) id));
    builder.addDoubleProperty("ElementID", this::getElementID, (double id)->setElementID((int) id));
  }

  public Command getAutonomousCommand() {
    return this.follow;  
  }
}

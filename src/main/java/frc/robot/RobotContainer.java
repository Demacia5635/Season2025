// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.commands.ArmCommand;
import frc.robot.robot1.arm.commands.ArmDrive;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.leds.Robot1Strip;
import frc.robot.leds.subsystems.LedManager;
import frc.robot.utils.CommandController;
import frc.robot.utils.LogManager;
import frc.robot.utils.CommandController.ControllerType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{

  public static RobotContainer robotContainer;
  public static LedManager ledManager;
  public static CommandController controller;
  public static boolean isRed = true;

  public static Chassis chassis;  
  public static Arm arm;
  public static Gripper gripper;
  public static Robot1Strip robot1Strip;

  public static Drive drive;

  public static ArmCalibration armCalibration;
  public static ArmCommand armCommand;
  public static ArmDrive armDrive;
  public static Command armSetStateTesting;

  public static Grab grab;
  public static Drop drop;

  public POSITION fieldPosition;

  public FieldTarget fieldTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new AutoUtils();
    new LogManager();
    ledManager = new LedManager();
    controller = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);

    SmartDashboard.putData("Reef", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Reef");
        builder.addDoubleProperty("Position", ()-> fieldTarget.position.ordinal(), index-> fieldTarget.position = POSITION.values()[(int)index]);
        builder.addDoubleProperty("Element Position", ()-> fieldTarget.elementPosition.ordinal(), index-> fieldTarget.elementPosition = ELEMENT_POSITION.values()[(int)index]);
        builder.addDoubleProperty("Level", ()-> fieldTarget.level.ordinal(), index-> fieldTarget.level = LEVEL.values()[(int)index]);
      }
    });

    configureSubsytems();
    configureCommands();
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * This function start all the subsytems.
   * Put here all the subsystems you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureSubsytems() {
    chassis = new Chassis();
    arm = new Arm();
    gripper = new Gripper();
    robot1Strip = new Robot1Strip(arm, gripper);
  }

 


  /**
   * This function start all the commands.
   * Put here all the commands you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureCommands() {
    //drive = new Drive(chassis, controller);
    drive = new Drive(chassis, controller);

    armCalibration = new ArmCalibration(arm);
    armCommand = new ArmCommand(arm);
    // armDrive = new ArmDrive(arm, controller);
    armSetStateTesting = new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.TESTING)).ignoringDisable(true);

    grab = new Grab(gripper);
    drop = new Drop(gripper);
  }

  /**
   * This function set all the default commands to the subsystems.
   * set all the default commands of the subsytems.
   * This function is called at the robot container constractor
   */
  private void configureDefaultCommands() {
    chassis.setDefaultCommand(drive);
    arm.setDefaultCommand(armCommand);
  }


  private void configureBindings() {

    controller.leftButton().onTrue(new ArmCalibration(arm));
   
    controller.rightButton().onTrue(new Drop(gripper));
    controller.leftBumper().onTrue(getDisableInitCommand());

    controller.povUp().onTrue(new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.L3_TOUCHING)));
    controller.rightBumper().onTrue(new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.L2_TOUCHING)));
    controller.rightSetting().onTrue(new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.STARTING)));

    controller.upButton().onTrue(new InstantCommand(()-> chassis.setYaw(Rotation2d.fromDegrees(0)), chassis).withTimeout(0.25));

    //controller.downButton().onTrue();
    
  }

  public static boolean isRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::setIsRed);
  }

  /**
   * This command is schedules at the start of teleop.
   * Look in {@link Robot} for more details.
   * @return the ommand that start at the start at enable
   */
  public Command getEnableInitCommand() {
    return armCalibration;
  }

  /**
   * This command is schedules at the start of disable.
   * Put here all the stop functions of all the subsytems and then add them to the requirments
   * This insures that the motors do not keep their last control mode earlier and moves uncontrollably.
   * Look in {@link Robot} for more details.
   * @return the command that runs at disable
   */
  public Command getDisableInitCommand() {
    Command initDisableCommand = new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
    }, chassis, arm, gripper
    ).ignoringDisable(true);
    initDisableCommand.setName("Init Disable Command");
    return initDisableCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

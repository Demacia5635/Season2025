// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.robot1.arm.commands.ArmCommand;
import frc.robot.robot1.arm.commands.ArmDrive;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.utils.LogManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static CommandXboxController controller;

  public static Arm arm;
  public static Gripper gripper;

  public static ArmCalibration armCalibration;
  public static ArmCommand armCommand;
  public static ArmDrive armDrive;
  public static Command armSetStateTesting;

  public static Grab grab;
  public static Drop drop;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new LogManager();
    controller = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

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
    arm = new Arm();
    gripper = new Gripper();
  }

  /**
   * This function start all the commands.
   * Put here all the commands you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureCommands() {
    armCalibration = new ArmCalibration(arm);
    armCommand = new ArmCommand(arm);
    armDrive = new ArmDrive(arm, controller);
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
    arm.setDefaultCommand(armCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.a().onTrue(armDrive);
    controller.y().onTrue(armSetStateTesting);
    controller.x().onTrue(armCalibration);
    controller.leftBumper().onTrue(getDisableInitCommand());
    controller.a().onTrue(grab);
    controller.b().onTrue(drop);
    controller.leftBumper().onTrue(getDisableInitCommand());
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
      arm.stop();
      gripper.stop();
    }, arm, gripper
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

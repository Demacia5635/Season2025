// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.robot2.arm.commands.ArmCommand;
import frc.robot.robot2.arm.commands.ArmDrive;
import frc.robot.robot2.arm.subsystems.Arm;
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
  public static CommandController driverController;
  public static CommandController operatorController;
  public static boolean isRed = true;
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;
  public static Arm arm;

  Timer timer;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    robotContainer = this;
    new LogManager();
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);
    timer = new Timer();
    LogManager.addEntry("Timer",()-> 15-timer.get());

    configureSubsytems();
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
  }

  /**
   * This function set all the default commands to the subsystems.
   * set all the default commands of the subsytems.
   * This function is called at the robot container constractor
   */
  private void configureDefaultCommands() {
    arm.setDefaultCommand(new ArmCommand(arm));
  }


  private void configureBindings() {
  }

  public static boolean isRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean isComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::isComp, RobotContainer::setIsComp);
  }

  /**
   * This command is schedules at the start of teleop.
   * Look in {@link Robot} for more details.
   * @return the ommand that start at the start at enable
   */
  public Command getEnableInitCommand() {
    return null;
    // return armCalibration;
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
      timer.stop();
    }
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
    timer.reset();
    timer.start();
    return null;
    // return (new ArmCalibration(arm).andThen(new ArmCommand(arm))).alongWith(new L2AlgaeL3(chassis, hasRemovedFromLog));  
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.gripper.constants.GripperConstants.DropConstants;
import frc.robot.robot1.gripper.subsystems.Gripper;

/**
 * the drop command.
 * <br>
 * </br>
 * this command move the gripper at specific speed until the coral was passed by
 * the sensor and then was not in the gripper
 */
public class Drop extends Command {

  /** the gripper subsystem */
  private final Gripper gripper;
  /** a variable to check if the sensor has seen the coral */
  private boolean hasSeenCoral;

  private Command settingArmDown;

  private Timer timer;

  /**
   * creates a new Drop command
   * <br>
   * </br>
   * the function configure has seen coral to false and add the gripper to
   * requirments
   * 
   * @param gripper
   */
  public Drop(Gripper gripper) {
    this.gripper = gripper;
    hasSeenCoral = false;
    settingArmDown = new WaitUntilCommand(()-> RobotContainer.chassis.getPose().getTranslation().getDistance(RobotContainer.isRed ? AutoUtils.redReefCenter : AutoUtils.blueReefCenter) >= 1.6).andThen(
      new InstantCommand(()-> {if (RobotContainer.arm.state != ARM_ANGLE_STATES.CORAL_STATION) RobotContainer.arm.setState(ARM_ANGLE_STATES.STARTING);}));
    timer = new Timer();
    addRequirements(gripper);
  }

  /**
   * this function is called at the start of the command
   * <br>
   * </br>
   * this function is initializing the has seen coral to false
   */
  @Override
  public void initialize() {
    RobotContainer.robot1Strip.setDrop();
    hasSeenCoral = false;
    timer.start();
  }

  /**
   * this funcition is called every cycle of the command
   * <br>
   * </br>
   * this function is giving the gripper motor a specific power and check if the
   * gripper seeing coral
   */
  @Override
  public void execute() {
    gripper.setPower(DropConstants.DROP_POWER);
    if (gripper.isCoralUpSensor() || gripper.isCoralDownSensor()) {
      hasSeenCoral = true;
    }
  }

  /**
   * this function is called after the command have finished
   * <br>
   * </br>
   * this function stop the gripper
   */
  @Override
  public void end(boolean interrupted) {
    gripper.stop();
    timer.stop();
    timer.reset();
    if (!interrupted && !DriverStation.isAutonomous()) {
      settingArmDown.schedule();
    }
  }

  /**
   * this function checks if the command have finished
   * <br>
   * </br>
   * the condition is if the gripepr is not seeing coral and had seen coral
   * 
   * @return the condition
   */
  @Override
  public boolean isFinished() {
    return (!gripper.isCoralDownSensor() && hasSeenCoral) || timer.hasElapsed(0.8);
  }
}

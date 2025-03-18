// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.robot2.gripper.constants.GripperConstants.GrabConstants;
import frc.robot.robot2.gripper.subsystems.Gripper;

/**
 * the grab command.
 * <br>
 * </br>
 * this command move the gripper at a sepcific power until the coral is inside
 * the gripper
 */
public class Grab extends Command {

  /** the gripper subsystem */
  private Gripper gripper;

  /**
   * creates a new grab command,
   * <br>
   * </br>
   * this function add the gripper to requirments
   * 
   * @param gripper the wanted gripper
   */
  public Grab(Gripper gripper) {
    this.gripper = gripper;
    addRequirements(gripper);
  }

  /**
   * this function is called at the start of the command
   * <br>
   * </br>
   * this function does nothing
   */
  @Override
  public void initialize() {}

  /**
   * this function is called every cycle of the command
   * <br>
   * </br>
   * the function move the gripper at a specific speed
   */
  @Override
  public void execute() {
    gripper.setPower(GrabConstants.FEED_POWER);
  }

  /**
   * this function is called after the command had finished
   * <br>
   * </br>
   * the function stop the gripper
   */
  @Override
  public void end(boolean interrupted) {
    gripper.stop();
    
    if (!interrupted) {
      new AlignCoral(gripper).schedule();
    }
  }

  /**
   * this function is called after the command had finished
   * <br>
   * </br>
   * the condition is if the coral is in the gripper
   * 
   * @return the condition to finish the command
   */
  @Override
  public boolean isFinished() {
    return gripper.isCoralDownSensor();
    // return false;
    // return gripper.isCoral();
    // return !gripper.isCoral() && hasSeen;
  }
}

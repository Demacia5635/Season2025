// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.gripper.constants.GripperConstants.GrabConstants;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.utils.LogManager;

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
  private boolean hasSeen;
  private int upCount = 0;
  private int downCount = 0;
  private Timer timer;
  private boolean notSeenAnySensor;

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
    this.hasSeen = false;
    this.timer = new Timer();
    addRequirements(gripper);
  }

  /**
   * this function is called at the start of the command
   * <br>
   * </br>
   * this function does nothing
   */
  @Override
  public void initialize() {
    hasSeen = false;
    upCount = 0;
    downCount = 0;
    notSeenAnySensor = false;
  }

  /**
   * this function is called every cycle of the command
   * <br>
   * </br>
   * the function move the gripper at a specific speed
   */
  @Override
  public void execute() {
    if (gripper.isCoralDownSensor() && !timer.isRunning()) {
      // timer.start();
      hasSeen = true;
      gripper.stop();
    }

    // if (timer.hasElapsed(0.05) && !hasSeen) {
    //   LogManager.log("hasSeen = true");
    //   hasSeen = true;
    // }

    if (!hasSeen) {
      gripper.setPower(GrabConstants.FEED_POWER);
    } else {
      if (gripper.isCoral()) {
        gripper.stop();
        downCount = 0;
        upCount = 0;
      } else if (gripper.isCoralUpSensor()) {
        downCount++;
        upCount = 0;
        if (notSeenAnySensor) {
          notSeenAnySensor = false;
          gripper.stop();
        }
      } else if (gripper.isCoralDownSensor()) {
        upCount++;
        downCount = 0;
        if (notSeenAnySensor) {
          notSeenAnySensor = false;
          gripper.stop();
        }
      } else {
        gripper.setPower(GrabConstants.DOWN_POWER);
        notSeenAnySensor = true;
      }
      
      if (upCount >= 4) {
        gripper.setPower(GrabConstants.UP_POWER);
      } else if (downCount >= 4) {
        gripper.setPower(GrabConstants.DOWN_POWER);
      }
    }
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
    timer.stop();
    timer.reset();
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
    return false;
    // return gripper.isCoral();
    // return !gripper.isCoral() && hasSeen;
  }
}

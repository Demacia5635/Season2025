// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.arm.constants.ArmConstants.ArmAngleMotorConstants;
import frc.robot.robot1.arm.constants.ArmConstants.CalibrationConstants;
import frc.robot.robot1.arm.subsystems.Arm;

/**
 * command that calibrate the arm
 * <br></br>
 * the arm will move forward for couple for a little under a second and then move backwards until it hit the limit switch
 */
public class ArmCalibration extends Command {

  /** The arm subsytem */
  private Arm arm;
  /** timer to check if at the start the wanted seconds have elapsed and can the arm move down */
  Timer timer;

  /**
   * creates a new calibration command
   * <br></br>
   * this function configure the timer and add the arm to the requirements
   * @param arm the wanted arm to calibrate
   */
  public ArmCalibration(Arm arm) {
    this.arm = arm;
    timer = new Timer();
    addRequirements(arm);
  }

  /**
   * the function that is called at the start of the command
   * <br></br>
   * the function start the timer put the arm angle motor at brake and start to move forward
   */
  @Override
  public void initialize() {
    timer.start();
    arm.armAngleNeutralMode(true);
    arm.armAngleMotorSetPower(CalibrationConstants.ARM_ANGLE_START_POWER);
  }

  /**
   * the function that is called every cycle of the command
   * <br></br>
   * the function check if the timer has elapsed and then move the arm backwards
   */
  @Override
  public void execute() {
    if (timer.hasElapsed(CalibrationConstants.TIME_TO_CHANGE_POWER)) {
      arm.armAngleMotorSetPower(CalibrationConstants.ARM_ANGLE_POWER);
    }
  }

  /**
   * the function that is called after the command had finished
   * <br></br>
   * the function stop the arm and set the motor position to the base angle
   * <br></br>
   * then set the arm to calibrated and reset the timer
   */
  @Override
  public void end(boolean interrupted) {
    /* stop the arm */
    arm.stop();

    /* set the arm angle motor position to base angle and set the arm to calibrated */
    arm.armAngleSetPosition(ArmAngleMotorConstants.BASE_ANGLE);
    arm.hadCalibrated();

    /* reset the timer */
    timer.stop();
    timer.reset();
  }

  /**
   * the function that is called to check if the command have finished
   * <br></br>
   * the condition is if the limit switch of the arm is closed
   */
  @Override
  public boolean isFinished() {
    return arm.getArmAngleLimit();
  }
}

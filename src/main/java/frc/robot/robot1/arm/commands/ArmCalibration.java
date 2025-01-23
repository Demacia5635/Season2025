// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.arm.constants.ArmConstants.ArmAngleMotorConstants;
import frc.robot.robot1.arm.constants.ArmConstants.CalibrationConstants;
import frc.robot.robot1.arm.subsystems.Arm;

public class ArmCalibration extends Command {

  private Arm arm;
  Timer timer;

  /** Creates a new Calibration. */
  public ArmCalibration(Arm arm) {
    this.arm = arm;
    timer = new Timer();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    arm.armAngleNeutralMode(true);
    arm.armAngleMotorSetPower(CalibrationConstants.ARM_ANGLE_START_POWER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(CalibrationConstants.TIME_TO_CHANGE_POWER)) {
      arm.armAngleMotorSetPower(CalibrationConstants.ARM_ANGLE_POWER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();

    arm.armAngleSetPosition(ArmAngleMotorConstants.BASE_ANGLE);
    arm.isCalibrated = true;

    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getArmAngleLimit();
  }
}

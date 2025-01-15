// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.arm.constants.ArmConstants.ANGLES;
import frc.robot.robot1.arm.subsystems.Arm;

public class ArmCommand extends Command {
  private Arm arm;
  public ANGLES state;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm) {
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case L2:
        break;

      case L3:
        break;

      case CORAL_STATION:
        break;

      case TESTING:
        break;

      default:

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

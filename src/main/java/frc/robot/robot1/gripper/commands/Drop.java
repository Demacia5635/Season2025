// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.gripper.constants.GripperConstants.DropConstants;
import frc.robot.robot1.gripper.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drop extends Command {

  Gripper gripper;
  boolean hasSeenCoral;

  /** Creates a new Drop. */
  public Drop(Gripper gripper) {
    this.gripper = gripper;
    hasSeenCoral = false;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeenCoral = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.setPower(DropConstants.DROP_POWER);
    if (gripper.isCoral()) {
      hasSeenCoral = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !gripper.isCoral() && hasSeenCoral;
  }
}

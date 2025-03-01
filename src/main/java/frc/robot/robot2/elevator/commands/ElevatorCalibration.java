// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot2.elevator.ElevatorConstants.CalibrationConstants;
import frc.robot.robot2.elevator.subsystem.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCalibration extends Command {

  private final Elevator elevator;
  private boolean hasReechedBottom = false;

  /** Creates a new ElevatorCalibration. */
  public ElevatorCalibration(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasReechedBottom = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPower(CalibrationConstants.POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();

    elevator.calibrated();
    elevator.setPosition(
      hasReechedBottom
      ? CalibrationConstants.BOTTOM_HEIGHT
      : CalibrationConstants.TOP_HEIGHT
    );
    hasReechedBottom = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    hasReechedBottom = elevator.hasReachedBottom();
    return elevator.hasReachedBottom() || elevator.hasReachedTop();
  }
}

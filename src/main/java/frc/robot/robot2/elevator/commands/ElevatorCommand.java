// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot2.elevator.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.robot2.elevator.subsystem.Elevator;
import frc.robot.utils.LogManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {

  private final Elevator elevator;

  private double testHeight;

  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator) {
    this.elevator = elevator;
    SmartDashboard.putData("Elevator Command", this);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setNeutalMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (elevator.getState()) {
      case L4:
      case L3:
      case L2:
      case CORAL_STATION:
      case STARTING:
        elevator.setPositionVoltage(elevator.getState().HEIGHT);
        break;

      case TESTING:
          break;

      case IDLE:
        elevator.stop();
        break;

      default:
        LogManager.log("Illegul Elevator State", AlertType.kError);
        elevator.stop();
        elevator.setState(ELEVATOR_STATE.IDLE);
        break;
    }
  }

  public double getTestHeight() {
    return testHeight;
  }

  public void setTestHeight(double height) {
    this.testHeight = height;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Elevator test height", this::getTestHeight, this::setTestHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

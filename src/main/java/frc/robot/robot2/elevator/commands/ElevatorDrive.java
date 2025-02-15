package frc.robot.robot2.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot2.elevator.subsystem.Elevator;

public class ElevatorDrive extends Command {
    Elevator elevator;
    CommandXboxController controller;

    public ElevatorDrive(Elevator elevator, CommandXboxController controller) {
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPower(controller.getLeftY() * -0.5);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}

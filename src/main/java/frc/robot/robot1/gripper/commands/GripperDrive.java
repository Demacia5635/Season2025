package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.utils.CommandController;

public class GripperDrive extends Command {

    private final Gripper gripper;
    private final CommandController controller;

    public GripperDrive(Gripper gripper, CommandController controller) {
        this.gripper = gripper;
        this.controller = controller;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setPower(
            (controller.getLeftTrigger() - controller.getRightTrigger()) * 0.3
        );
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
        if (!interrupted) {
            new AlignCoral(gripper).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return !controller.downButton().getAsBoolean();
    }
}

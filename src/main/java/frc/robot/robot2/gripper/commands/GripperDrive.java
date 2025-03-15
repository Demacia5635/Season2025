package frc.robot.robot2.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot2.gripper.subsystems.Gripper;
import frc.robot.utils.CommandController;

public class GripperDrive extends Command {

    private final Gripper gripper;
    private final CommandController controller;
    private boolean leftBoost;
    private boolean rightBoost;

    public GripperDrive(Gripper gripper, CommandController controller) {
        this.gripper = gripper;
        this.controller = controller;
        this.leftBoost = leftBoost;
        this.rightBoost = rightBoost;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        leftBoost = false;
        rightBoost = false;
    }

    @Override
    public void execute() {
        leftBoost = controller.leftBumper().getAsBoolean();
        rightBoost = controller.rightBumper().getAsBoolean();

        gripper.setPower(
            ((leftBoost ? controller.getLeftTrigger() * 2 : controller.getLeftTrigger()) - 
            (rightBoost ? controller.getRightTrigger() * 2 : controller.getRightTrigger())) * 0.4
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

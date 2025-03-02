package frc.robot.robot2.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot2.gripper.subsystems.Gripper;

public class GrabOrDrop extends Command {
    
    private final Gripper gripper;

    public GrabOrDrop(Gripper gripper) {
        this.gripper = gripper;
    }

    public void end(boolean interrupted) {
        if (gripper.isCoral()) {
            new Drop(gripper).schedule();
        } else {
            new Grab(gripper).schedule();
        }
    }

    public boolean isFinished() {
        return true;
    }
}

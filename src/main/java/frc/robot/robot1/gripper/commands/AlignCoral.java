package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.gripper.constants.GripperConstants.GrabConstants;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class AlignCoral extends Command{

    private final Gripper gripper;
    
    public AlignCoral(Gripper gripper) {
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if (!gripper.isCoralUpSensor() && !gripper.isCoralDownSensor()) {
            gripper.stop();
        } else if (gripper.isCoralUpSensor()) {
            gripper.setPower(GrabConstants.DOWN_POWER);
        } else if (gripper.isCoralDownSensor()) {
            gripper.setPower(GrabConstants.UP_POWER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    @Override
    public boolean isFinished() {
        return gripper.isCoral() || (!gripper.isCoralDownSensor() && !gripper.isCoralUpSensor());
    }
}

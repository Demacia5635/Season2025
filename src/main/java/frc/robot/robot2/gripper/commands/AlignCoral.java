
package frc.robot.robot2.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.robot2.gripper.constants.GripperConstants.AlignCoralConstants;
import frc.robot.robot2.gripper.subsystems.Gripper;

public class AlignCoral extends Command {

    private final Gripper gripper;

    private double upCount;
    private double downCount;
    private boolean notSeenAnySensor;

    public AlignCoral(Gripper gripper) {
        this.gripper = gripper;
        this.upCount = 0;
        this.downCount = 0;
        this.notSeenAnySensor = false;
        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        upCount = 0;
        downCount = 0;
        notSeenAnySensor = false;
    }

    @Override
    public void execute() {
        if (gripper.isCoral()) {
            gripper.stop();
            downCount = 0;
            upCount = 0;
        } else if (gripper.isCoralUpSensor()) {
            downCount++;
            upCount = 0;
            if (notSeenAnySensor) {
                notSeenAnySensor = false;
                gripper.stop();
            }
        } else if (gripper.isCoralDownSensor()) {
            upCount++;
            downCount = 0;
            if (notSeenAnySensor) {
                notSeenAnySensor = false;
                gripper.stop();
            }
        } else {
            gripper.setPower(AlignCoralConstants.DOWN_POWER);
            notSeenAnySensor = true;
        }
      
        if (upCount > 3) {
            gripper.setPower(AlignCoralConstants.UP_POWER);
        } else if (downCount > 3) {
            gripper.setPower(AlignCoralConstants.DOWN_POWER);
        }
    }

    @Override
    public void end(boolean interrupted) {
       gripper.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

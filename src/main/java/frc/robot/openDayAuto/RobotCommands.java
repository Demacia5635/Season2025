package frc.robot.openDayAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class RobotCommands {
    private static final Arm ARM;
    private static final Gripper GRIPPER;

    static {
        ARM = RobotContainer.arm;
        GRIPPER = RobotContainer.gripper;
    }

    public static final Command grab() {
        Command cmd = new InstantCommand(() ->
            ARM.setState(ARM_ANGLE_STATES.CORAL_STATION)
        ).andThen(new Grab(GRIPPER));
        
        cmd.addRequirements(ARM, GRIPPER);
        cmd = cmd.withTimeout(5);
        return cmd;
    }

    public static final Command putL2() {
        Command cmd = new InstantCommand(() -> 
            ARM.setState(ARM_ANGLE_STATES.L2)
        ).andThen(new Drop(GRIPPER));

        cmd.addRequirements(GRIPPER, ARM);
        cmd = cmd.withTimeout(5);
        return cmd;
    }

    public static final Command putL3() {
        Command cmd = new InstantCommand(() -> 
            ARM.setState(ARM_ANGLE_STATES.L3)
        ).andThen(new Drop(GRIPPER));

        cmd.addRequirements(GRIPPER, ARM);
        cmd = cmd.withTimeout(5);
        return cmd;
    }

    public static final Command removeBottomAlgae() {
        return AutoUtils.removeAlgae(false, true);
    }

    public static final Command removeTopAlgae() {
        return AutoUtils.removeAlgae(true, true);
    }
}

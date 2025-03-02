package frc.robot.robot2.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot2.arm.subsystems.Arm;
import frc.robot.utils.CommandController;

/**
 * command to control the arm with controller
 * <br>
 * </br>
 * the command is for testing to move the arm using the controller
 */
public class ArmDrive extends Command {

    /** the controller */
    private final CommandController controller;
    /** the arm subsystem */
    private final Arm arm;

    /** the arm angle motor power later changed by the controller */
    private double armAnglePower;
    /** the gripper angle motor power later changed by the controller */
    private double gripperAnglePower;

    /**
     * creates a new arm drive command
     * <br>
     * </br>
     * the function configure the arm angle power and gripper angle power and add
     * arm to requirments
     * 
     * @param arm        the wanted arm to drive
     * @param controller the controller to drive with
     */
    public ArmDrive(Arm arm, CommandController controller) {
        this.arm = arm;
        this.controller = controller;

        armAnglePower = 0;
        gripperAnglePower = 0;

        addRequirements(arm);
    }

    /**
     * the function that is called at the start of the command
     * <br>
     * </br>
     * the function does nothing
     */
    @Override
    public void initialize() {
    }

    /**
     * the function that is called every cycle of the command
     * <br>
     * </br>
     * the function takes the controller joystick checks, deadband them and lower
     * them and give to the arm
     */
    @Override
    public void execute() {
        armAnglePower = controller.getLeftY() * -0.7;
        gripperAnglePower = controller.getRightY() * -0.4 + (armAnglePower / 8);

        arm.setPower(armAnglePower, gripperAnglePower);
    }

    /**
     * the function that is called after the command had finished
     * <br>
     * </br>
     * the function stop the arm
     */
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    /**
     * the function is called to check if the command have finished
     * <br>
     * </br>
     * there is no condition the command will go forever until the drive is
     * activating diffrent command
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}

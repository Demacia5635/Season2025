package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot1.arm.subsystems.Arm;

/**
 * command to control the arm with controller
 * <br>
 * </br>
 * the command is for testing to move the arm using the controller
 */
public class ArmDrive extends Command {

    /** the controller */
    CommandXboxController controller;
    /** the arm subsystem */
    Arm arm;

    /** the arm angle motor power later changed by the controller */
    double armAnglePower;
    /** the gripper angle motor power later changed by the controller */
    double gripperAnglePower;

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
    public ArmDrive(Arm arm, CommandXboxController controller) {
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
        armAnglePower = Math.abs(controller.getLeftY()) <= 0.2 ? 0 : controller.getLeftY() * -0.5;
        gripperAnglePower = Math.abs(controller.getRightY()) <= 0.2 ? 0 : controller.getRightY() * -0.3;

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

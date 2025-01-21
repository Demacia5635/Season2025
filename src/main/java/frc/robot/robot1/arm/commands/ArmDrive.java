package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot1.arm.subsystems.Arm;

public class ArmDrive extends Command{
    CommandXboxController controller;
    Arm arm;

    public ArmDrive(Arm arm, CommandXboxController controller) {
        this.arm = arm;
        this.controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        arm.armAngleMotorSetPower(controller.getLeftY() * -0.5);
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

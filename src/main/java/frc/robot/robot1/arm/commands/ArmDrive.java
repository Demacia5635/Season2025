package frc.robot.robot1.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.robot1.arm.subsystems.Arm;

public class ArmDrive extends Command{
    CommandXboxController controller;
    Arm arm;

    double armAnglePower;
    double gripperAnglePower;

    public ArmDrive(Arm arm, CommandXboxController controller) {
        this.arm = arm;
        this.controller = controller;
        
        armAnglePower = 0;
        gripperAnglePower = 0;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armAnglePower = Math.abs(controller.getLeftY()) <= 0.2 ? 0 : controller.getLeftY() * -0.5;
        gripperAnglePower = Math.abs(controller.getRightY()) <= 0.2 ? 0 : controller.getRightY() * -0.3;
        
        arm.setPower(armAnglePower, gripperAnglePower);
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

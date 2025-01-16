package frc.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.Utils;

public class Drive extends Command {
    private Chassis chassis;
    private CommandXboxController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;

    public Drive(Chassis chassis, CommandXboxController controller) {
        this.chassis = chassis;
        this.controller = controller;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        double joyX = Utils.deadband(controller.getLeftY(), 0.13) * direction;
        double joyY = Utils.deadband(controller.getLeftX(), 0.13) * direction;
        
        // Calculate r]otation from trigger axes
        double rot = (Utils.deadband(controller.getLeftTriggerAxis(), 0.13)
            - Utils.deadband(controller.getRightTriggerAxis(), 0.13));
        
        double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
        double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
        double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_OMEGA_VELOCITY * Math.signum(rot);
        speeds = new ChassisSpeeds(velX, velY,velRot);
        chassis.setVelocities(speeds);
    }
}

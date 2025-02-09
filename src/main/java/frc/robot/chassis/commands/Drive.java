package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.utils.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.CommandController;
import frc.robot.utils.Utils;

public class Drive extends Command {
    private Chassis chassis;
    private CommandController controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;

    public Drive(Chassis chassis, CommandController controller) {
        this.chassis = chassis;
        this.controller = controller;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        isRed = chassis.isRed();
        direction = isRed ? 1 : -1;
        double joyX = controller.getLeftY() * direction;
        double joyY = controller.getLeftX() * direction;
        
        // Calculate r]otation from trigger axes
        double rot = controller.getLeftTrigger() - controller.getRightTrigger();
        
        double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
        double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
        double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_OMEGA_VELOCITY * Math.signum(rot);
        
        speeds = new ChassisSpeeds(velX, velY,velRot);
        
        // Translation2d RightJoyVector = Utils.getStickVector(controller);
        // Rotation2d sticAngle = RightJoyVector.getAngle().unaryMinus().plus(Rotation2d.fromDegrees(90));
        // if (!isRed){
        //     sticAngle = sticAngle.plus(Rotation2d.fromDegrees(180));
        // }
        
        // if(RightJoyVector.getNorm() > 0.3){
        //     chassis.setVelocitiesRotateToAngle(speeds, sticAngle);
        // }else{ 
        chassis.setVelocities(speeds);
        // }
    }
}

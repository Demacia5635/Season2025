package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.constants.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.Utils;

public class Drive extends Command {
    private Chassis chassis;
    private CommandXboxController controller;
    private CommandPS5Controller ps5Controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;

    public Drive(Chassis chassis, CommandXboxController controller) {
        this.chassis = chassis;
        this.controller = controller;

        addRequirements(chassis);
    }

    public Drive(Chassis chassis, CommandPS5Controller controller) {
        this.chassis = chassis;
        this.ps5Controller = controller;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        if (controller != null) {

            isRed = chassis.isRed();
            direction = isRed ? 1 : -1;
            double joyX = Utils.deadband(controller.getLeftY(), 0.13) * direction;
            double joyY = Utils.deadband(controller.getLeftX(), 0.13) * direction;
            
            // Calculate r]otation from trigger axes
            double rot = (Utils.deadband((controller.getLeftTriggerAxis()) , 0.13)
            - Utils.deadband((controller.getRightTriggerAxis()), 0.13));
            
            double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
            double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
            double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_OMEGA_VELOCITY * Math.signum(rot);
            
            speeds = new ChassisSpeeds(velX, velY,velRot);
            
            Translation2d RightJoyVector = Utils.getStickVector(controller);
            Rotation2d sticAngle = RightJoyVector.getAngle().unaryMinus().plus(Rotation2d.fromDegrees(90));
            if (!isRed){
                sticAngle = sticAngle.plus(Rotation2d.fromDegrees(180));
            }
            if(RightJoyVector.getNorm() > 0.3){
                chassis.setVelocitiesRotateToAngle(speeds, sticAngle);
            }else{
                chassis.setVelocities(speeds);
            }
        } else {
            isRed = chassis.isRed();
            direction = isRed ? 1 : -1;
            double joyX = Utils.deadband(ps5Controller.getLeftY(), 0.015) * direction;
            double joyY = Utils.deadband(ps5Controller.getLeftX(), 0.015) * direction;
            
            // Calculate r]otation from trigger axes
            double rot = (Utils.deadband((ps5Controller.getL2Axis() + 1) / 2d, 0.03)
            - Utils.deadband((ps5Controller.getR2Axis() + 1) /2d, 0.03));
            
            double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
            double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
            double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_OMEGA_VELOCITY * Math.signum(rot);
            
            speeds = new ChassisSpeeds(velX, velY,velRot);
            
            Translation2d RightJoyVector = Utils.getStickVector(ps5Controller);
            Rotation2d sticAngle = RightJoyVector.getAngle().unaryMinus().plus(Rotation2d.fromDegrees(90));
            if (!isRed){
                sticAngle = sticAngle.plus(Rotation2d.fromDegrees(180));
            }
            if(RightJoyVector.getNorm() > 0.3){
                chassis.setVelocitiesRotateToAngle(speeds, sticAngle);
            }else{
                chassis.setVelocities(speeds);
            }
        }
    }
}

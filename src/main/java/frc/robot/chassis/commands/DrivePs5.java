package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.constants.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.Utils;

public class DrivePs5 extends Command {
    private Chassis chassis;
    private PS5Controller controller;
    private double direction;
    private boolean isRed;
    private ChassisSpeeds speeds;

    public DrivePs5(Chassis chassis, PS5Controller controller) {
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
        double rot = (Utils.deadband((controller.getR2Axis() + 1) / 2, 0.05)
        - Utils.deadband((controller.getL2Axis() + 1) / 2, 0.07));
        
        double velX = Math.pow(joyX, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyX);
        double velY = Math.pow(joyY, 2) * ChassisConstants.MAX_DRIVE_VELOCITY * Math.signum(joyY);
        double velRot = Math.pow(rot, 2) * ChassisConstants.MAX_OMEGA_VELOCITY * Math.signum(rot);

        speeds = new ChassisSpeeds(velX, velY,velRot);

        Translation2d RightJoyVector = Utils.getStickVectorPs5(controller);
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

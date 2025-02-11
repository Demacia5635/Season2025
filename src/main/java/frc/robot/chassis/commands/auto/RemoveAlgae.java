package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

public class RemoveAlgae extends Command {

    private final Chassis chassis;
    private final FieldTarget target;
    private final Timer steerTimer;
    private final Timer driveBackTimer;

    public RemoveAlgae(Chassis chassis, FieldTarget fieldTarget) {
        this.chassis = chassis;
        this.target = fieldTarget;
        this.steerTimer = new Timer();
        this.driveBackTimer = new Timer();
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setVelocities(new ChassisSpeeds(0, 0, 
        0.5 * (target.position == POSITION.A || target.position == POSITION.B || target.position == POSITION.F ? 1 : -1)));
        steerTimer.start();
    }

    @Override
    public void execute() {
        if (steerTimer.hasElapsed(1)) {
            chassis.setRobotRelVelocities(new ChassisSpeeds(
                -2, 0, 0
            ));
            steerTimer.stop();
            steerTimer.reset();
            driveBackTimer.start();
        } 
        LogManager.log(driveBackTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
        steerTimer.stop();
        steerTimer.reset();
        driveBackTimer.stop();
        driveBackTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return driveBackTimer.hasElapsed(0.3);
    }
}

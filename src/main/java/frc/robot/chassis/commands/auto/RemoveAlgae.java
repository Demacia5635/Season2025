package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
    private boolean isAuto;
    private boolean isAlgaeRight;

    public RemoveAlgae(Chassis chassis, FieldTarget fieldTarget, boolean isAlgaeRight) {
        this.isAuto = DriverStation.isAutonomous();
        this.chassis = chassis;
        this.target = fieldTarget;
        this.steerTimer = new Timer();
        this.driveBackTimer = new Timer();
        this.isAlgaeRight = isAlgaeRight;
        addRequirements(chassis);
    }

    public RemoveAlgae(Chassis chassis, FieldTarget fieldTarget) {
        this(chassis, fieldTarget, fieldTarget.position == POSITION.C || fieldTarget.position == POSITION.D || fieldTarget.position == POSITION.E );
    }

    @Override
    public void initialize() {
        chassis.setVelocities(new ChassisSpeeds(-0.5, 0, 
        4.5 * (isAlgaeRight ? -1 : 1)));
        steerTimer.start();

        isAuto = DriverStation.isAutonomous();
    }

    @Override
    public void execute() {
        if (steerTimer.hasElapsed(0.5)) {
            chassis.setRobotRelVelocities(new ChassisSpeeds(
                -2, 0, 0
            ));
            steerTimer.stop();
            steerTimer.reset();
            driveBackTimer.start();
        } 
    }

    @Override
    public void end(boolean interrupted) {
        if(!isAuto){
            
            chassis.stop();
            steerTimer.stop();
            steerTimer.reset();
            driveBackTimer.stop();
            driveBackTimer.reset();
    
        }
    }

    @Override
    public boolean isFinished() {
        return driveBackTimer.hasElapsed(0.3) || (isAuto && steerTimer.hasElapsed(0.3));
    }
}

package frc.robot.robot1.climb.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.climb.subsystem.Climb;

public class ClimbUntilSensor extends Command {

    private final Climb climb;
    private boolean isSensorAtStart;
    private boolean hasMovedFromSensor;

    public ClimbUntilSensor(Climb climb) {
        this.climb = climb;
        this.isSensorAtStart = false;
        this.hasMovedFromSensor = false;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        isSensorAtStart = climb.getLimit();
        hasMovedFromSensor = false;
    }

    @Override
    public void execute() {
        climb.setClimbPower(1);
        if (isSensorAtStart && !climb.getLimit()) {
            hasMovedFromSensor = true; 
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    @Override
    public boolean isFinished() {
        return (!isSensorAtStart && climb.getLimit()) 
        || (isSensorAtStart && hasMovedFromSensor && climb.getLimit());
    }


}

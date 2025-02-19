package frc.robot.Path.Trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.vision.utils.VisionConstants;

public class ChangeReefToClosest extends Command {

    private final Chassis chassis;

    public ChangeReefToClosest(Chassis chassis) {
        this.chassis = chassis;
    }

    POSITION closestTagToChassis() {
        POSITION closestReef = null;
        double closestReefToRobot = Double.MAX_VALUE;
        for (int i = 0; i < 6; i++) {
            double norm = chassis.getPose().getTranslation()
                    .minus(VisionConstants.O_TO_TAG[POSITION.values()[i].getId()]).getNorm();
            if (norm < closestReefToRobot) {
                closestReefToRobot = norm;
                closestReef = POSITION.values()[i];
            }
        }
        return closestReef;
    }

    public void initialize() {
        POSITION closestReef = closestTagToChassis();
        if (closestReef != null) {
            RobotContainer.scoringTarget.position = closestReef;
        }
    };

    public void end(boolean interrupted) {
        if (!interrupted) {
            new FollowTrajectory(chassis, true).schedule();
        }
    };

    public boolean isFinished() {
        return true;
    };
}

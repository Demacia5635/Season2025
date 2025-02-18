package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;

public class ReefWidget implements Sendable {
    private FieldTarget scoringTarget;
    private static ReefWidget reefWidget;

    public ReefWidget() {
        this.scoringTarget = RobotContainer.scoringTarget;
    }

    public static ReefWidget getInstance() {
        if (reefWidget == null) reefWidget = new ReefWidget();
        return reefWidget;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Reef");
        builder.addDoubleProperty("Position", () -> scoringTarget.position.ordinal(), index -> {
            if (!isFeeding(index, 0)) {
                scoringTarget.position = POSITION.values()[(int) index];
            }
        });
        builder.addDoubleProperty("Element Position", () -> scoringTarget.elementPosition.ordinal(), index -> {
            if (!isFeeding(index, 1)) {
                scoringTarget.elementPosition = ELEMENT_POSITION.values()[(int) index];
            }
        });
        builder.addDoubleProperty("Level", () -> scoringTarget.level.ordinal(), index -> {
            if (!isFeeding(index, 2)) {
                scoringTarget.level = LEVEL.values()[(int) index];
            }
        });
    }

    private boolean isFeeding(double index, int element) {
        switch (element) {
            case 0:
                return index == 6 || index == 7;
            case 1:
                return index == 3;
            case 2:
                return index == 2;
            default:
                return false;
        }
    }
}

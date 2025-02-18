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
    private boolean hasChanged;

    public ReefWidget() {
        this.scoringTarget = RobotContainer.scoringTarget;
        this.hasChanged = false;
    }

    public static ReefWidget getInstance() {
        if (reefWidget == null)
            reefWidget = new ReefWidget();
        return reefWidget;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Reef");

        builder.addDoubleProperty("Position", () -> !hasChanged ? -1 : scoringTarget.position.ordinal(),
                index -> onSelect(index, 0));
        builder.addDoubleProperty("Element Position", () -> !hasChanged ? -1 : scoringTarget.elementPosition.ordinal(),
                index -> onSelect(index, 1));
        builder.addDoubleProperty("Level", () -> !hasChanged ? -1 : scoringTarget.level.ordinal(),
                index -> onSelect(index, 2));
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

    private void onSelect(double index, int elementType) {
        if (!isFeeding(index, elementType)) {
            hasChanged = true;
            switch (elementType) {
                case 0:
                    scoringTarget.position = POSITION.values()[(int) index];
                    break;

                case 1:
                    scoringTarget.elementPosition = ELEMENT_POSITION.values()[(int) index];
                    break;

                case 2:
                    scoringTarget.level = LEVEL.values()[(int) index];
                    break;

                default:
                    break;
            }

        }
    }
}

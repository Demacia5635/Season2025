package frc.robot.robot1;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.subsystems.Arm;

public class RobotCoastOrBrake extends Command {
    private static boolean isNowBrake = true;

    private final Chassis chassis;
    private final Arm arm;

    public RobotCoastOrBrake(Chassis chassis, Arm arm) {
        this.arm = arm;
        this.chassis = chassis;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        RobotContainer.robot1Strip.setUserButton();

        chassis.setNeutralMode(!isNowBrake);
        arm.armAngleNeutralMode(!isNowBrake);
        arm.gripperAngleNeutralMode(!isNowBrake);

        isNowBrake = !isNowBrake;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

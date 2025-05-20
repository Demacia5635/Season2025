package frc.robot.openDayAuto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.subsystems.Chassis;

public class Turn {
    private static final Chassis CHASSIS;
    private static final PathPoint DUMMY_POINT;

    static {
        CHASSIS = RobotContainer.chassis;
        DUMMY_POINT = PathPoint.kZero;
    }

    public static Command turnLeft(double angle) {
        Command cmd = new Command() {
            Command cmd;
            
            @Override
            public void initialize() {
                cmd = new FollowTrajectory(CHASSIS, new ArrayList<>() {{
                    add(DUMMY_POINT);
                    add(new PathPoint(new Pose2d(CHASSIS.getPose().getTranslation(), CHASSIS.getGyroAngle().plus(Rotation2d.fromDegrees(angle)))));
                }}, CHASSIS.getGyroAngle().plus(Rotation2d.fromDegrees(angle)));
                cmd.schedule(); 
            }

            @Override
            public void end(boolean interrupted) {
                CHASSIS.stop();
                Timer timer = new Timer();
                while (!timer.hasElapsed(0.5));
            }

            @Override
            public boolean isFinished() {
                return cmd.isFinished();
            }
        };
        
        cmd.addRequirements(CHASSIS);
        return cmd;
    }

    public static Command turnRight(double angle) {
        return turnLeft(-angle);
    }
}

package frc.robot.openDayAuto;

import java.util.ArrayList;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.subsystems.Chassis;

public class Move {
    private static final Chassis CHASSIS;
    private static final PathPoint DUMMY_POINT;

    static {
        CHASSIS = RobotContainer.chassis;
        DUMMY_POINT = PathPoint.kZero;
    }

    public static Command moveForward(double meters) {
        Command cmd = new Command() {
            Command cmd;

            @Override
            public Set<Subsystem> getRequirements() {
                return new Set<Subsystem>() {
                    
                };
            }
            
            @Override
            public void initialize() {
                cmd = new FollowTrajectory(CHASSIS, new ArrayList<>() {{
                    add(DUMMY_POINT);
                    add(new PathPoint(new Pose2d(CHASSIS.getPose().getTranslation().plus(new Translation2d(0, meters)), CHASSIS.getGyroAngle())));
                }}, CHASSIS.getGyroAngle());
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

    public static Command moveLeft(double meters) {
        Command cmd = new Command() {
            Command cmd;
            
            @Override
            public void initialize() {
                cmd = new FollowTrajectory(CHASSIS, new ArrayList<>() {{
                    add(DUMMY_POINT);
                    add(new PathPoint(new Pose2d(CHASSIS.getPose().getTranslation().plus(new Translation2d(meters, 0)), CHASSIS.getGyroAngle())));
                }}, CHASSIS.getGyroAngle());
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

    public static Command moveBackwards(double meters) {
        return moveForward(-meters);
    }

    public static Command moveRight(double meters) {
        return moveLeft(-meters);
    }
}

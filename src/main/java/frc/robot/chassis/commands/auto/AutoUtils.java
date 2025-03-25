package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.Path.Utils.Segment;

import static frc.robot.chassis.utils.ChassisConstants.*;
import static frc.robot.vision.utils.VisionConstants.*;

import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;

public class AutoUtils {

    static Chassis chassis;
    static double maxVel = MAX_DRIVE_VELOCITY;
    static Translation2d cornerOffsetLeft = new Translation2d(0.480, 0.93212813);
    static Translation2d cornerOffsetRight = new Translation2d(0.480, -0.93212813);
    public static final Translation2d blueReefCenter = new Translation2d(4.490, 4.035);
    public static final Translation2d redReefCenter = new Translation2d(13.058902, 4.035);
    public static final int[] reefCams = { 0, 3 };

    public AutoUtils() {
        chassis = RobotContainer.chassis;
    }

    public static Segment[] REEF_SEGMENTS = {
            getSegments(6),
            getSegments(7),
            getSegments(8),
            getSegments(9),
            getSegments(10),
            getSegments(11)
    };

    public static Segment getSegments(int elementTag) {
        return new Segment(O_TO_TAG[elementTag].plus(cornerOffsetRight.rotateBy(TAG_ANGLE[elementTag])),
                O_TO_TAG[elementTag].plus(cornerOffsetLeft.rotateBy(TAG_ANGLE[elementTag])));
    }

    public boolean isSeeTag(int id, int cameraId, double distance) {
        return chassis.isSeeTag(id, cameraId, distance);
    }

    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle, 0);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new PathPoint(from.getX() + x, from.getY() + y, Rotation2d.fromDegrees(angle), radius);
    }

    public static Command leave() {
        return new RunCommand(() -> chassis.setVelocities(
                new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3);
    }

    public static Command goTo(Pose2d wantedPose, double threshold) {
        return goTo(wantedPose, threshold, new int[0], new int[0]);
    }

    public static Command removeAlgae(boolean algaeTop, boolean isRight) {
        if (algaeTop)
            return new InstantCommand(() -> RobotContainer.arm.setState(ARM_ANGLE_STATES.AFTER_ALGAE_TOP))
                    .andThen(new WaitCommand(0.1).andThen(new RunCommand(() -> chassis.setRobotRelVelocities(
                            new ChassisSpeeds(-3, 0, 0)), chassis)).withTimeout(0.6),
                            new InstantCommand(() -> chassis.stop(), chassis));

        return new InstantCommand(() -> RobotContainer.arm.setState(ARM_ANGLE_STATES.AFTER_ALGAE_BOTTOM))
                .andThen(new WaitCommand(0.3)).andThen(
                        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(-3, 0, DriverStation.isAutonomous() ? isRight ? -4 : 4 : 0)),
                                chassis).withTimeout(0.5),
                                new InstantCommand(() -> chassis.stop(), chassis));
    }

    public static Command goTo(Pose2d wantedPose, double threshold, int[] cameraID, int[] tagID) {

        return new Command() {

            @Override
            public void execute() {
                chassis.goTo(wantedPose, threshold, true);
            }

            @Override
            public boolean isFinished() {
                if (cameraID.length == 0 || tagID.length == 0)
                    return chassis.getPose().getTranslation().getDistance(wantedPose.getTranslation()) <= threshold;
                else {
                    for (int cID : cameraID) {
                        for (int tID : tagID) {
                            if (chassis.isSeeTag(tID, cID, Double.MAX_VALUE)) {
                                return true;
                            }
                        }
                    }
                    return false;
                }
            }
        };

    }

}

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.PathFollow;
import frc.robot.PathFollow.Util.pathPoint;
import static frc.robot.chassis.ChassisConstants.*;
import frc.robot.chassis.subsystems.Chassis;

public class AutoUtils {


    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static double maxVel = MAX_DRIVE_VELOCITY;
    static double maxAceel = DRIVE_ACCELERATION;
    static pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    static boolean isRed = RobotContainer.isRed();


    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }
    
    public static pathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle,0);
    }

    public static pathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new pathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),radius,false);
    }

    public static  Command goTo(pathPoint point) {
        return goTo(point, maxVel, true);
    }
    public static  Command goTo(pathPoint point, double maxV) {
        return goTo(point, maxV, true);
    }
    public static  Command goToRotate(pathPoint point, double maxV, double rate) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxV, maxAceel,
         0, false).setAutoRotate(rate);
    }

    public static Command goToMultiple(pathPoint[] points, double maxVel){
        return new PathFollow(chassis, points, maxVel, maxAceel, 0, false);
    }
    public static  Command goTo(pathPoint point, double maxv, boolean toSpeaker) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, 0, toSpeaker);
    }
    public static  Command goTo(pathPoint point, double maxv, boolean toSpeaker, double endV) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, endV, toSpeaker);
    }


    public static Command leave() {
        return new RunCommand(()-> chassis.setVelocities(
            new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3);
    }

    
}

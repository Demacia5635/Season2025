package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.Path.Utils.Segment;

import static frc.robot.chassis.utils.ChassisConstants.*;
import static frc.robot.vision.utils.VisionConstants.*;

import java.util.HashMap;

import org.opencv.features2d.FlannBasedMatcher;

import frc.robot.chassis.subsystems.Chassis;

public class AutoUtils {


    static Chassis chassis = RobotContainer.chassis;
    static double maxVel = MAX_DRIVE_VELOCITY;
    static double maxAceel = DRIVE_ACCELERATION;
    static Translation2d cornerOffsetLeft = new Translation2d(0, 0.48);
    static Translation2d cornerOffsetRight = new Translation2d(0., -0.48);
    
    

    public static Segment[] REEF_SEGMENTS = {
        getSegments(6),
        getSegments(7),
        getSegments(8),
        getSegments(9),
        getSegments(10),
        getSegments(11)
    };

    public static Segment getSegments(int elementTag){
        return new Segment(O_TO_TAG[elementTag].plus(cornerOffsetRight.rotateBy(TAG_ANGLE[elementTag])), O_TO_TAG[elementTag].plus(cornerOffsetLeft.rotateBy(TAG_ANGLE[elementTag])));
    }
    public boolean isSeeTag(int id, int cameraId, double distance){
        return chassis.isSeeTag(id, cameraId, distance);
    }

    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }
    
    public static PathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle,0);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new PathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),radius);
    }

    public static Command leave() {
        return new RunCommand(()-> chassis.setVelocities(
            new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3);
    }
    

    
}

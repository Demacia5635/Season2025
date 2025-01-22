package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.PathFollow;
import frc.robot.PathFollow.Util.PathPoint;
import static frc.robot.chassis.ChassisConstants.*;
import static frc.robot.vision.utils.VisionConstants.*;

import java.util.HashMap;

import frc.robot.chassis.subsystems.Chassis;

public class AutoUtils {


    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static double maxVel = MAX_DRIVE_VELOCITY;
    static double maxAceel = DRIVE_ACCELERATION;
    static PathPoint dummyPoint = new PathPoint(0, 0, new Rotation2d(), 0, false);
    static boolean isRed = RobotContainer.isRed();
    static Translation2d offset = new Translation2d(1.5, 0);
// for red
    public enum FIELD_ELEMENTS{
        FEEDER_LEFT, FEEDER_RIGHT, A_LEFT, A_RIGHT, A_CENTER, B_LEFT, B_RIGHT, B_CENTER, C_LEFT, C_RIGHT, C_CENTER, D_LEFT, D_RIGHT, D_CENTER, E_LEFT, E_RIGHT, E_CENTER, F_LEFT, F_RIGHT, F_CENTER

    }
    public static boolean isLeft(FIELD_ELEMENTS element){
        return element == FIELD_ELEMENTS.FEEDER_LEFT || element == FIELD_ELEMENTS.A_LEFT
         || element == FIELD_ELEMENTS.B_LEFT || element == FIELD_ELEMENTS.C_LEFT
         || element == FIELD_ELEMENTS.D_LEFT || element == FIELD_ELEMENTS.E_LEFT
         || element == FIELD_ELEMENTS.F_LEFT;
    }
    
    public static HashMap<FIELD_ELEMENTS, PathPoint> fieldElements = new HashMap<>();
    static {
        fieldElements.put(FIELD_ELEMENTS.FEEDER_LEFT, getElement(1, offset));
        fieldElements.put(FIELD_ELEMENTS.FEEDER_RIGHT, getElement(2, offset));
        fieldElements.put(FIELD_ELEMENTS.A_LEFT, getElement(6, offset));
        fieldElements.put(FIELD_ELEMENTS.A_RIGHT, getElement(6, offset));
        fieldElements.put(FIELD_ELEMENTS.A_CENTER, getElement(6, offset));
        fieldElements.put(FIELD_ELEMENTS.B_LEFT, getElement(7, offset));
        fieldElements.put(FIELD_ELEMENTS.B_RIGHT, getElement(7, offset));
        fieldElements.put(FIELD_ELEMENTS.B_CENTER, getElement(7, offset));
        fieldElements.put(FIELD_ELEMENTS.C_LEFT, getElement(8, offset));
        fieldElements.put(FIELD_ELEMENTS.C_RIGHT, getElement(8, offset));
        fieldElements.put(FIELD_ELEMENTS.C_CENTER, getElement(8, offset));
        fieldElements.put(FIELD_ELEMENTS.D_LEFT, getElement(9, offset));
        fieldElements.put(FIELD_ELEMENTS.D_RIGHT, getElement(9, offset));
        fieldElements.put(FIELD_ELEMENTS.D_CENTER, getElement(9, offset));
        fieldElements.put(FIELD_ELEMENTS.E_LEFT, getElement(10, offset));
        fieldElements.put(FIELD_ELEMENTS.E_RIGHT, getElement(10, offset));
        fieldElements.put(FIELD_ELEMENTS.E_CENTER, getElement(10, offset));
        fieldElements.put(FIELD_ELEMENTS.F_LEFT, getElement(11, offset));
        fieldElements.put(FIELD_ELEMENTS.F_RIGHT, getElement(11, offset));
        fieldElements.put(FIELD_ELEMENTS.F_CENTER, getElement(11, offset));

    }

    public static PathPoint getElement(int elementTag, Translation2d ofset){
        Translation2d originToTag = O_TO_TAG[elementTag];
        ofset = ofset.rotateBy(TAG_ANGLE[elementTag]);
        return new PathPoint(originToTag.plus(ofset), TAG_ANGLE[elementTag]);
    }

    public AutoUtils(){
        fieldElements.put(FIELD_ELEMENTS.FEEDER_LEFT, new PathPoint(new Translation2d(15.5, 1.8), Rotation2d.fromDegrees(-55)));
    }



    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }
    
    public static PathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle,0);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new PathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),radius,false);
    }

    public static  Command goTo(PathPoint point) {
        return goTo(point, maxVel, true);
    }
    public static  Command goTo(PathPoint point, double maxV) {
        return goTo(point, maxV, true);
    }
    public static  Command goToRotate(PathPoint point, double maxV, double rate) {
        return new PathFollow(chassis, new PathPoint[] { dummyPoint, point }, maxV, maxAceel,
         0, false).setAutoRotate(rate);
    }

    public static Command goToMultiple(PathPoint[] points, double maxVel, Rotation2d finalAngle, boolean isConstVel){
        return new PathFollow(points, finalAngle, maxVel, isConstVel);
    }
    public static  Command goTo(PathPoint point, double maxv, boolean toSpeaker) {
        return new PathFollow(chassis, new PathPoint[] { dummyPoint, point }, maxv, maxAceel, 0, toSpeaker);
    }
    public static  Command goTo(PathPoint point, double maxv, boolean toSpeaker, double endV) {
        return new PathFollow(chassis, new PathPoint[] { dummyPoint, point }, maxv, maxAceel, endV, toSpeaker);
    }
    public static Command goToLine(Pose2d point, Pose2d pose, double vel){
        Translation2d diff = point.getTranslation().minus(pose.getTranslation());
        return new RunCommand(()->chassis.setVelocitiesRotateToAngle(new ChassisSpeeds(vel * diff.getAngle().getCos(), vel * diff.getAngle().getSin(), 0), point.getRotation()));
    }


    public static Command leave() {
        return new RunCommand(()-> chassis.setVelocities(
            new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3);
    }

    
}

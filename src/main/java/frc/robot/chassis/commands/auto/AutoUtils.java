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
import frc.robot.PathFollow.Util.Segment;

import static frc.robot.chassis.constants.ChassisConstants.*;
import static frc.robot.vision.VisionConstants.*;

import java.util.HashMap;

import frc.robot.chassis.subsystems.Chassis;

public class AutoUtils {


    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static double maxVel = MAX_DRIVE_VELOCITY;
    static double maxAceel = DRIVE_ACCELERATION;
    static PathPoint dummyPoint = new PathPoint(0, 0, new Rotation2d(), 0, false);
    static boolean isRed = RobotContainer.isRed();
    static Translation2d strateOffset = new Translation2d(2, 0);
    static Translation2d practicalOffsetFeeder = new Translation2d(0.9, 0);
    static Translation2d cornerOffsetLeft = new Translation2d(0., 0.48);
    static Translation2d cornerOffsetRight = new Translation2d(0., -0.48);
    
    
    // for red
    public enum FIELD_POSITION{
        FEEDER_LEFT, FEEDER_RIGHT, A, B, C, D, E, F
    }
    public enum REEF_SEGMENTS{
        A,B,C,D,E,F
    }
    public enum ELEMENT{
        ALGAE, CORAL_RIGHT, CORAL_LEFT, FEEDER
    }

    public enum LEVEL{
        L2, L3, ALGAE_BOTTOM, ALGAE_TOP, FEEDER
    }
    
    public static HashMap<FIELD_POSITION, PathPoint> fieldElements = new HashMap<>();

    public static HashMap<FIELD_POSITION, PathPoint> fieldElementsPractical = new HashMap<>();
    

    public static Segment[] REEF_SEGMENTS = {
        getSegments(6),
        getSegments(7),
        getSegments(8),
        getSegments(9),
        getSegments(10),
        getSegments(11)
    };
    public static FIELD_POSITION[] FIELD_POSITIONS = {
        FIELD_POSITION.A,
        FIELD_POSITION.A,
        FIELD_POSITION.B,
        FIELD_POSITION.C,
        FIELD_POSITION.D,
        FIELD_POSITION.E,
        FIELD_POSITION.F,
        FIELD_POSITION.FEEDER_LEFT,
        FIELD_POSITION.FEEDER_RIGHT

    };

    public static ELEMENT[] ELEMENTS = {
        ELEMENT.CORAL_LEFT,
        ELEMENT.CORAL_RIGHT,
        ELEMENT.ALGAE,
        ELEMENT.FEEDER
    };

    public static PathPoint[] REEF_POINTS = {
        fieldElements.get(FIELD_POSITION.A),
        fieldElements.get(FIELD_POSITION.B),
        fieldElements.get(FIELD_POSITION.C),
        fieldElements.get(FIELD_POSITION.D),
        fieldElements.get(FIELD_POSITION.E),  
        fieldElements.get(FIELD_POSITION.F)
    };

    public static PathPoint getElement(int elementTag, Translation2d ofset){
        Translation2d originToTag = O_TO_TAG[elementTag];
        ofset = ofset.rotateBy(TAG_ANGLE[elementTag]);
        return new PathPoint(originToTag.plus(ofset), TAG_ANGLE[elementTag].plus(Rotation2d.fromDegrees(180)), 0.2);
    }

    public static Segment getSegments(int elementTag){
        return new Segment(O_TO_TAG[elementTag].plus(cornerOffsetRight.rotateBy(TAG_ANGLE[elementTag])), O_TO_TAG[elementTag].plus(cornerOffsetLeft.rotateBy(TAG_ANGLE[elementTag])), false);
    }

    public static double L2DIST = -0.84;
    public static double L3DIST = -0.95;
    public static double ALGAE_TOP_DIST = 0;
    public static double ALGAE_BOTTOM_DIST = 0;

    public static double FEEDER_DIST = -(0.73);
    public static double RIGHT_SIDE_DIST = 0.20;
    public static double LEFT_SIDE_DIST = -0.20;



    public AutoUtils(){
        fieldElements.put(FIELD_POSITION.FEEDER_LEFT, getElement(1, strateOffset));
        fieldElements.put(FIELD_POSITION.FEEDER_RIGHT, getElement(2, strateOffset));
        fieldElements.put(FIELD_POSITION.A, getElement(6, strateOffset));
        fieldElements.put(FIELD_POSITION.B, getElement(7, strateOffset));
        fieldElements.put(FIELD_POSITION.C, getElement(8, strateOffset));
        fieldElements.put(FIELD_POSITION.D, getElement(9, strateOffset));
        fieldElements.put(FIELD_POSITION.E, getElement(10, strateOffset));
        fieldElements.put(FIELD_POSITION.F, getElement(11, strateOffset));


        
        fieldElementsPractical.put(FIELD_POSITION.FEEDER_LEFT, getElement(1, practicalOffsetFeeder));
        fieldElementsPractical.put(FIELD_POSITION.FEEDER_RIGHT, getElement(2, practicalOffsetFeeder));
        fieldElementsPractical.put(FIELD_POSITION.A, getElement(6, strateOffset));
        fieldElementsPractical.put(FIELD_POSITION.B, getElement(7, strateOffset));
        fieldElementsPractical.put(FIELD_POSITION.C, getElement(8, strateOffset));
        fieldElementsPractical.put(FIELD_POSITION.D, getElement(9, strateOffset));
        fieldElementsPractical.put(FIELD_POSITION.E, getElement(10, strateOffset));
        fieldElementsPractical.put(FIELD_POSITION.F, getElement(11, strateOffset));


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
    public static Command goToMultiple(PathPoint[] points, double maxVel, Rotation2d finalAngle, boolean isConstVel, boolean isPrecise){
        return new PathFollow(points, finalAngle, maxVel, isConstVel, isPrecise);
    }
    public static Command goToMultiple(PathPoint[] points, double maxVel, Rotation2d finalAngle, boolean isConstVel, boolean isPrecise, FIELD_POSITION toGoElement, AlignToTag alignToTag){
        return new PathFollow(points, finalAngle, maxVel, isConstVel, isPrecise, toGoElement, alignToTag);
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

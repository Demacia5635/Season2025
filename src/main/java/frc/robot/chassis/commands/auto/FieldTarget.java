// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;
import static frc.robot.vision.utils.VisionConstants.O_TO_TAG;
import static frc.robot.vision.utils.VisionConstants.TAG_ANGLE;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

public class FieldTarget {

    public static final Translation2d approachOffset = new Translation2d(1, 0);
    public static final Translation2d approachOffsetAlgaeRight = new Translation2d(1, 0.6);
    public static final Translation2d approachOffsetAlgaeLeft = new Translation2d(1, -0.6);
    public static final Translation2d reefOffsetLeft = new Translation2d(0, -0.11);
    public static final Translation2d reefOffsetRight = new Translation2d(0, 0.25);
    public static Translation2d intakeOffset = new Translation2d(0.66, 0);
    public static Translation2d topAlgeaRightOffset = new Translation2d(0.4,0.4);
    public static Translation2d topAlgeaLeftOffset = new Translation2d(0.4,-0.4);
    public static Translation2d bottomAlgeaRightOffset = new Translation2d(0.46, 0.47);
    public static Translation2d bottomAlgeaLeftOffset = new Translation2d(0.46, -0.47);
    public static final Translation2d l2Offset = new Translation2d(0.64, 0);
    public static final Translation2d l3Offset = new Translation2d(0.5, 0);
    public static final Translation2d realLeftReefOffset = new Translation2d(-0.05,-0.16);
    public static final Translation2d realRightReefOffset = new Translation2d(-0.05,0.16);

    public static Translation2d l2Left = new Translation2d(0.58, -0.12);
    public static Translation2d l2Right = new Translation2d(0.57, 0.24);
    public static Translation2d l3Left = new Translation2d(0.47, -0.12);
    public static Translation2d l3Right = new Translation2d(0.47, 0.24);

    public POSITION position;
    public ELEMENT_POSITION elementPosition;
    public LEVEL level;


    public static final FieldTarget kFeederLeft = new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER);
    public static final FieldTarget kFeederRight = new FieldTarget(POSITION.FEEDER_RIGHT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER);

    public FieldTarget(POSITION position, ELEMENT_POSITION elementPosition, LEVEL level){
        this.position = position;
        this.elementPosition = elementPosition;
        this.level = level;

    }

    
    public static PathPoint[] REEF_POINTS = new PathPoint[]{
        new FieldTarget(POSITION.A, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.B, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.C, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.D, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.E, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.F, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER).getApproachingPoint(),
    };




    public enum ELEMENT_POSITION{
        CORAL_LEFT, CORAL_RIGHT, ALGEA, FEEDER
    }
    
    public enum LEVEL{
        L2,
        L3,
        FEEDER,
        ALGAE_BOTTOM,
        ALGAE_TOP
    }
    public enum POSITION{
        A(6, 19),
        B(7, 18), 
        C(8, 17),
        D(9, 22),
        E(10, 21),
        F(11, 20),
        FEEDER_LEFT(1, 13),
        FEEDER_RIGHT(2, 12);

        private int redId;
        private int blueId;
        POSITION(int redTagID, int blueTagID){
            this.redId = redTagID;
            this.blueId = blueTagID;
        }

        public int getId() {
            return RobotContainer.isRed? redId:blueId;
        }
        
        public PathPoint getApproachPoint(Translation2d offset) {
            return getElement(getId(), offset);
        }
        
    }

    public Translation2d getPole(){
        return getElement(position.getId(), new Translation2d(0.6, 0).plus(elementPosition == ELEMENT_POSITION.CORAL_LEFT ? realLeftReefOffset : realRightReefOffset)).getTranslation();
    }

    private Translation2d getApproachOffsetByChassis(){
        if (RobotContainer.chassis == null) return Translation2d.kZero;
        Translation2d diffVector = getFinishPoint().getTranslation().minus(RobotContainer.chassis.getPose().getTranslation());
        if(Math.abs(diffVector.getAngle().getRadians()) <= Math.toRadians(15)) return Translation2d.kZero;
        if(diffVector.getAngle().getRadians() < 0) return new Translation2d(0, -0.3);
        return new Translation2d(0, 0.3);
    }

    public PathPoint getApproachingPoint(boolean isAlgaeRight){
        if(elementPosition == ELEMENT_POSITION.ALGEA){
            if(isAlgaeRight){
                return position.getApproachPoint(approachOffsetAlgaeLeft);
            }
            else{
                return position.getApproachPoint(approachOffsetAlgaeRight);
            }
        } else if (elementPosition == ELEMENT_POSITION.CORAL_LEFT) {
            return position.getApproachPoint(approachOffset.plus(realLeftReefOffset));
        } else if (elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {
            return position.getApproachPoint(approachOffset.plus(realRightReefOffset));
        } else {
            return position.getApproachPoint(approachOffset);
        }
    }

    public PathPoint getApproachingPoint() {
        return getApproachingPoint(position == POSITION.C || position == POSITION.D || position == POSITION.E);
    }

    public PathPoint getFinishPoint(boolean isAlgaeRight){
        return getElement(position.getId(), getCalcOffset(isAlgaeRight));
    }

    public PathPoint getFinishPoint() {
        return getElement(position.getId(), getCalcOffset());
    }

    public PathPoint getReefPole(){
        return getElement(position.getId(), (elementPosition == ELEMENT_POSITION.CORAL_LEFT) ? realLeftReefOffset : realRightReefOffset);
    }

    public Translation2d getCalcOffset(boolean isAlgaeRight) {
        boolean isScoring = level == LEVEL.L2 || level == LEVEL.L3;
        Translation2d calculatedOffset = new Translation2d();

        if(isScoring){
            if (level == LEVEL.L2) {
                if (elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {
                    calculatedOffset = l2Right;
                } else {
                    calculatedOffset = l2Left;
                }
            } else if (level == LEVEL.L3) {
                if (elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {
                    calculatedOffset = l3Right;
                } else {
                    calculatedOffset = l3Left;
                }
            }
        }
        else{
            if(level == LEVEL.FEEDER){
                calculatedOffset = intakeOffset;
            }
            else if(level == LEVEL.ALGAE_TOP){
                if(isAlgaeRight){
                    calculatedOffset = topAlgeaLeftOffset;
                }
                else{
                    calculatedOffset = topAlgeaRightOffset;
                }
            }
            else if(level == LEVEL.ALGAE_BOTTOM){
                if(isAlgaeRight){
                    calculatedOffset = bottomAlgeaLeftOffset;
                }
                else{
                    calculatedOffset = bottomAlgeaRightOffset;
                }
            }
        }

        return calculatedOffset;
    }

    public Translation2d getCalcOffset() {
        return getCalcOffset(position == POSITION.C || position == POSITION.D || position == POSITION.E);
    };

    public static PathPoint getElement(int elementTag){
        return getElement(elementTag, new Translation2d());
    }
    public static PathPoint getElement(int elementTag, Translation2d offset){
        Translation2d originToTag = O_TO_TAG[elementTag];
        offset = offset.rotateBy(TAG_ANGLE[elementTag]);
        return new PathPoint(originToTag.plus(offset), TAG_ANGLE[elementTag].plus(Rotation2d.kPi),0);
    }
   
    @Override
    public String toString() {
        return "Position: " + position
        + "\n" + "Element Position: " + elementPosition
        + "\n" + "Level: " + level;
    }
}

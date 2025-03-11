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
import frc.robot.vision.utils.VisionConstants;

public class FieldTarget {
    public static final Translation2d reefOffsetLeft = new Translation2d(0, -0.11);
    public static final Translation2d reefOffsetRight = new Translation2d(0, 0.25);
    public static Translation2d topAlgaeOffset = new Translation2d(0.54, -0.18);
    public static Translation2d bottomAlgaeOffset = new Translation2d(0.54, -0.18);

    public static Translation2d intakeOffset = new Translation2d(0.69, 0);
    public static Translation2d rightIntakeOffset = new Translation2d(0, 0.75);
    public static Translation2d leftIntakeOffset = new Translation2d(0, -0.75);

    public static final Translation2d l2Offset = new Translation2d(0.64, 0);
    public static final Translation2d l3Offset = new Translation2d(0.5, 0);
    public static final Translation2d realLeftReefOffset = new Translation2d(-0.05,-0.16);
    public static final Translation2d realRightReefOffset = new Translation2d(-0.05,0.16);

    public static Translation2d l2Left = new Translation2d(0.58, -0.14);
    public static Translation2d l2Right = new Translation2d(0.57, 0.22);
    public static Translation2d l3Left = new Translation2d(0.5, -0.14);
    public static Translation2d l3Right = new Translation2d(0.5, 0.22);


    public POSITION position;
    public ELEMENT_POSITION elementPosition;
    public LEVEL level;


    public static final FieldTarget kFeederLeft = new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER);
    public static final FieldTarget kFeederRight = new FieldTarget(POSITION.FEEDER_RIGHT, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER);

    public FieldTarget(POSITION position, ELEMENT_POSITION elementPosition, LEVEL level){
        this.position = position;
        this.elementPosition = elementPosition;
        this.level = level;

    }

    
    public static PathPoint[] REEF_POINTS = new PathPoint[]{
        new FieldTarget(POSITION.A, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.B, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.C, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.D, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.E, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
        new FieldTarget(POSITION.F, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getApproachingPoint(),
    };

    public enum ELEMENT_POSITION{
        CORAL_LEFT, 
        CORAL_MIDDLE,
        CORAL_RIGHT, 
        ALGEA, 
        FEEDER_LEFT,
        FEEDER_MIDDLE, 
        FEEDER_RIGHT;
    }

    public enum FEEDER_SIDE {
        CLOSE, MIDDLE, FAR
    }

    public static ELEMENT_POSITION getFeeder(FEEDER_SIDE feederSide, POSITION feederPosition) {
        if (feederSide == FEEDER_SIDE.MIDDLE) return ELEMENT_POSITION.FEEDER_MIDDLE;
        if (feederPosition == POSITION.FEEDER_LEFT) {
            if (feederSide == FEEDER_SIDE.CLOSE) {
                return ELEMENT_POSITION.FEEDER_LEFT;
            } else {
                return ELEMENT_POSITION.FEEDER_RIGHT;
            }
        } else if (feederPosition == POSITION.FEEDER_RIGHT) {
            if (feederSide == FEEDER_SIDE.FAR) {
                return ELEMENT_POSITION.FEEDER_RIGHT;
            } else {
                return ELEMENT_POSITION.FEEDER_LEFT;
            }
        } else {
            return ELEMENT_POSITION.FEEDER_MIDDLE;
        }
    }
    
    public enum LEVEL{
        L1,
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


    public PathPoint getApproachingPoint(){
        if(RobotContainer.chassis == null) return new PathPoint(new Translation2d(), new Rotation2d());
        Translation2d smartApproachOffset = getSmartApproachOffset(); 
        if(elementPosition == ELEMENT_POSITION.ALGEA){
            
                return position.getApproachPoint(new Translation2d(smartApproachOffset.getX(), smartApproachOffset.getY() + topAlgaeOffset.getY()));
            
        } else if (elementPosition == ELEMENT_POSITION.CORAL_LEFT) {
            return position.getApproachPoint(smartApproachOffset.plus(realLeftReefOffset));
        } else if (elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {
            return position.getApproachPoint(smartApproachOffset.plus(realRightReefOffset));
        } else if (elementPosition == ELEMENT_POSITION.FEEDER_LEFT) {
            return position.getApproachPoint(smartApproachOffset.plus(leftIntakeOffset));
        } else if (elementPosition == ELEMENT_POSITION.FEEDER_RIGHT) {
            return position.getApproachPoint(smartApproachOffset.plus(rightIntakeOffset));
        } else {
            return position.getApproachPoint(smartApproachOffset);
        }
    }

    public Translation2d getSmartApproachOffset(){
        Translation2d robotToTag = RobotContainer.chassis.getPose().getTranslation().minus(getFinishPoint().getTranslation()).rotateBy(VisionConstants.TAG_ANGLE[position.getId()].unaryMinus());
        
        double minDistance = 0.7;
        double maxDistance = 1.2;
        double algaeMinDistance = 1.2;
        double algaeMaxDistance = 1.6;
        
        
        if(elementPosition == ELEMENT_POSITION.ALGEA){
            if (robotToTag.getX() > algaeMinDistance && robotToTag.getX() < algaeMaxDistance) {
                return new Translation2d(robotToTag.getX(), 0);
            }
            
            if(robotToTag.getX() < algaeMinDistance) return new Translation2d(algaeMinDistance, 0);
            return new Translation2d(algaeMaxDistance, 0);
        }

        if (robotToTag.getX() > minDistance && robotToTag.getX() < maxDistance) {
            return new Translation2d(robotToTag.getX(), 0);
        }
        
        if(robotToTag.getX() < minDistance) return new Translation2d(minDistance, 0);
        return new Translation2d(maxDistance, 0);

    }
    public PathPoint getFinishPoint() {
        return getElement(position.getId(), getCalcOffset());
    }

    public PathPoint getReefPole(){
        return getElement(position.getId(), (elementPosition == ELEMENT_POSITION.CORAL_LEFT) ? realLeftReefOffset : realRightReefOffset);
    }

    public Translation2d getCalcOffset() {
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
                if (elementPosition == ELEMENT_POSITION.FEEDER_LEFT) {
                    calculatedOffset = intakeOffset.plus(leftIntakeOffset);
                } else if (elementPosition == ELEMENT_POSITION.FEEDER_MIDDLE) {
                    calculatedOffset = intakeOffset;
                } else if (elementPosition == ELEMENT_POSITION.FEEDER_RIGHT) {
                    calculatedOffset = intakeOffset.plus(rightIntakeOffset);
                }
            }
            else if(level == LEVEL.ALGAE_TOP){
                calculatedOffset = topAlgaeOffset;
            }
            else if(level == LEVEL.ALGAE_BOTTOM){
                calculatedOffset = bottomAlgaeOffset;
            }
        }

        return calculatedOffset;
    }

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

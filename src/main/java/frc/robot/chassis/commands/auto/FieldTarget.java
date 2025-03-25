// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;
import static frc.robot.vision.utils.VisionConstants.O_TO_TAG;
import static frc.robot.vision.utils.VisionConstants.TAG_ANGLE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.utils.Utils;
import frc.robot.vision.utils.VisionConstants;

public class FieldTarget {
    public static final Translation2d avoidReefOffset = new Translation2d(2, 0);

    public static final Translation2d reefOffsetLeft = new Translation2d(0, -0.11);
    public static final Translation2d reefOffsetRight = new Translation2d(0, 0.25);
    public static Translation2d topAlgaeOffset = new Translation2d(0.54, -0.18);
    public static Translation2d bottomAlgaeOffset = new Translation2d(0.54, -0.18);

    public static Translation2d intakeOffset = new Translation2d(0.77, 0);
    public static Translation2d rightIntakeOffset = new Translation2d(0, 0.75);
    public static Translation2d leftIntakeOffset = new Translation2d(0, -0.75);

    public static final Translation2d realLeftReefOffset = new Translation2d(-0.05,-0.16);
    public static final Translation2d realRightReefOffset = new Translation2d(-0.05,0.16);

    public static Translation2d l2Left = new Translation2d(0.62, -0.14);
    public static Translation2d l2Right = new Translation2d(0.62, 0.22);
    public static Translation2d l3Left = new Translation2d(0.5, -0.14);
    public static Translation2d l3Right = new Translation2d(0.5, 0.215);


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
        new FieldTarget(POSITION.A, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
        new FieldTarget(POSITION.B, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
        new FieldTarget(POSITION.C, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
        new FieldTarget(POSITION.D, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
        new FieldTarget(POSITION.E, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
        new FieldTarget(POSITION.F, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
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

    public PathPoint getReefAvoidPoint(){
        return getElement(position.getId(), avoidReefOffset);
    }

    public Translation2d getClosestFeeder() {
        double closestDistance = Double.MAX_VALUE;
        Translation2d closestOffset = Translation2d.kZero;
        for (int i = 0; i < 9; i++) {
            Translation2d offset = intakeOffset.plus(new Translation2d(0, 0.2032 * (i-4)));
            double dis = RobotContainer.chassis.getPose().getTranslation().getDistance(getElement(position.getId(), offset).getTranslation());
            if (closestDistance > dis) {
                closestDistance = dis;
                closestOffset = offset;
            }
        }
        return closestOffset;
    }

    public PathPoint getApproachingPoint(){
        if(RobotContainer.chassis == null) return PathPoint.kZero;
        Translation2d smartApproachOffset = getSmartApproachOffset(); 

    
        if(elementPosition == ELEMENT_POSITION.ALGEA){
            
                return position.getApproachPoint(new Translation2d(smartApproachOffset.getX(), smartApproachOffset.getY() + topAlgaeOffset.getY()));
            
        } else if (elementPosition == ELEMENT_POSITION.CORAL_LEFT) {
            return position.getApproachPoint(smartApproachOffset.plus(realLeftReefOffset));
        } else if (elementPosition == ELEMENT_POSITION.CORAL_RIGHT) {
            return position.getApproachPoint(smartApproachOffset.plus(realRightReefOffset));
        } else if (level == LEVEL.FEEDER){
            if(elementPosition == ELEMENT_POSITION.FEEDER_LEFT){
                position.getApproachPoint(intakeOffset.plus(leftIntakeOffset));
            }
            else if(elementPosition == ELEMENT_POSITION.FEEDER_RIGHT){
                position.getApproachPoint(intakeOffset.plus(rightIntakeOffset));
            }
            else 
                position.getApproachPoint(intakeOffset);
            
        }
        return position.getApproachPoint(smartApproachOffset);
        
    }

    public boolean isInRange(){
        Translation2d tagToRobot = RobotContainer.chassis.getPose().getTranslation().minus(getFinishPoint().getTranslation()).rotateBy(VisionConstants.TAG_ANGLE[position.getId()].unaryMinus());
        double tagToRobotAngle = Utils.angleFromTranslation2d(tagToRobot);
        
        return Math.abs(tagToRobotAngle) <= Math.toRadians(30);
    }

    public Translation2d getSmartApproachOffset(){
        Translation2d robotToTag = RobotContainer.chassis.getPose().getTranslation().minus(getFinishPoint().getTranslation()).rotateBy(VisionConstants.TAG_ANGLE[position.getId()].unaryMinus());
        
        double minDistance = 1;
        double maxDistance = 1.5;
        double algaeMinDistance = DriverStation.isAutonomous() ? 1.8 : 1.2;
        double algaeMaxDistance = DriverStation.isAutonomous() ? 2.2 : 1.6;
        

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
        Translation2d calculatedOffset = Translation2d.kZero;

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
        return getElement(elementTag, Translation2d.kZero);
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

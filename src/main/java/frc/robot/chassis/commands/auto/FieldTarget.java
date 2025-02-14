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
import frc.robot.utils.LogManager;

public class FieldTarget {

    public static final Translation2d approachOffset = new Translation2d(1.2, 0);
    public static final Translation2d approachOffsetAlgaeRight = new Translation2d(1.5, 1);
    public static final Translation2d approachOffsetAlgaeLeft = new Translation2d(1.5, -1);
    public static final Translation2d reefOffsetLeft = new Translation2d(0, -0.15);
    public static final Translation2d reefOffsetRight = new Translation2d(0, 0.27);
    public static final Translation2d intakeOffset = new Translation2d(0.775, 0);
    public static final Translation2d topAlgeaRightOffset = new Translation2d(0.50,0.5);
    public static final Translation2d topAlgeaLeftOffset = new Translation2d(0.50,-0.5);
    public static final Translation2d bottomAlgeaRightOffset = new Translation2d(0.55, 0.5);
    public static final Translation2d bottomAlgeaLeftOffset = new Translation2d(0.55, -0.5);
    public static final Translation2d l2Offset = new Translation2d(0.57, 0);
    public static final Translation2d l3Offset = new Translation2d(0.5, 0);
    public static final Translation2d reelLeftReefOffset = new Translation2d(-0.05,-0.16);
    public static final Translation2d reelRightReefOffset = new Translation2d(-0.05,0.16);
    public POSITION position;
    public ELEMENT_POSITION elementPosition;
    public LEVEL level;

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

    public PathPoint getApproachingPoint(boolean isAlgaeRight){
        if(elementPosition == ELEMENT_POSITION.ALGEA){
            if(isAlgaeRight){
                return position.getApproachPoint(approachOffsetAlgaeLeft);
            }
            else{
                return position.getApproachPoint(approachOffsetAlgaeRight);
            }
        } else {
            return position.getApproachPoint(approachOffset);
        }
    }

    public PathPoint getApproachingPoint() {
        return getApproachingPoint(position == POSITION.A || position == POSITION.B || position == POSITION.F);
    }

    public PathPoint getFinishPoint(boolean isAlgaeRight){
        return getElement(position.getId(), getCalcOffset(isAlgaeRight));
    }

    public PathPoint getFinishPoint() {
        return getElement(position.getId(), getCalcOffset());
    }

    public PathPoint getReefPole(){
        return getElement(position.getId(), (elementPosition == ELEMENT_POSITION.CORAL_LEFT) ? reelLeftReefOffset : reelRightReefOffset);
    }

    public Translation2d getCalcOffset(boolean isAlgaeRight) {
        boolean isScoring = level == LEVEL.L2 || level == LEVEL.L3;
        Translation2d calculatedOffset = new Translation2d();

        if(isScoring){
            Translation2d levelOffset = level == LEVEL.L3 ? l3Offset : l2Offset;
            Translation2d elementOffset = elementPosition == ELEMENT_POSITION.CORAL_LEFT ? reefOffsetLeft : reefOffsetRight;
            calculatedOffset = levelOffset.plus(elementOffset);
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
        return getCalcOffset(position == POSITION.A || position == POSITION.B || position == POSITION.F);
    };

    public static PathPoint getElement(int elementTag){
        return getElement(elementTag, new Translation2d());
    }
    public static PathPoint getElement(int elementTag, Translation2d offset){
        Translation2d originToTag = O_TO_TAG[elementTag];
        offset = offset.rotateBy(TAG_ANGLE[elementTag]);
        return new PathPoint(originToTag.plus(offset), TAG_ANGLE[elementTag].plus(Rotation2d.fromDegrees(180)));
    }
   
    @Override
    public String toString() {
        return "Position: " + position
        + "\n" + "Element Position: " + elementPosition
        + "\n" + "Level: " + level;
    }
}

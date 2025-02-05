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

public class FieldTarget {

    public static final Translation2d approachOffset = new Translation2d(1.5, 0);
    public static final Translation2d reefOffsetLeft = new Translation2d(-0.185, 0);
    public static Translation2d reefOffsetRight = new Translation2d(0.165, 0);

    private POSITION position;
    private ELEMENT_POSITION elementPosition;
    private LEVEL level;

    public FieldTarget(POSITION position, ELEMENT_POSITION elementPosition, LEVEL level){
        this.position = position;
        this.elementPosition = elementPosition;
        this.level = level;

    }

    
    public static PathPoint[] REEF_POINTS = new PathPoint[]{
        POSITION.A.getApproachPoint(),
        POSITION.B.getApproachPoint(),
        POSITION.C.getApproachPoint(),
        POSITION.D.getApproachPoint(),
        POSITION.E.getApproachPoint(),
        POSITION.F.getApproachPoint()
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
        
        public PathPoint getApproachPoint() {
            return getElement(getId(), approachOffset);
        }
        
    }

    public PathPoint getApproachingPoint(){
        return position.getApproachPoint();
    }

    public PathPoint getFinishPoint(){

        boolean isScoring = level == LEVEL.L2 || level == LEVEL.L3;
        Translation2d calculatedOffset = new Translation2d();

        if(isScoring){
            Translation2d levelOffset = level == LEVEL.L3 ? new Translation2d(0.48, 0) : new Translation2d(0.72, 0);
            Translation2d elementOffset = elementPosition == ELEMENT_POSITION.CORAL_LEFT ? reefOffsetLeft : reefOffsetRight;
            calculatedOffset = levelOffset.plus(elementOffset);
        }
        
        return getElement(position.getId(), calculatedOffset);
    }
    public static PathPoint getElement(int elementTag){
        return getElement(elementTag, new Translation2d());
    }
    public static PathPoint getElement(int elementTag, Translation2d offset){
        Translation2d originToTag = O_TO_TAG[elementTag];
        offset = offset.rotateBy(TAG_ANGLE[elementTag]);
        return new PathPoint(originToTag.plus(offset), TAG_ANGLE[elementTag].plus(Rotation2d.fromDegrees(180)), 0.15);
    }
   
}

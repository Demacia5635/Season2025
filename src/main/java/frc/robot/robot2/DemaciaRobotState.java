package frc.robot.robot2;

import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;

public enum DemaciaRobotState {
    L1(-2.55, -1.8, 0),
    L2(1.8, -0.5, 0),
    L3(0.8, -1.3, 0),
    L4(0.6, -1.5, 0.63),
     FEEDER(-2.51, -1.6, 0.5),
    STARTING(-2.569414874465336, -2.4, 0),
    TESTING(0,0,0),
    IDLE(0,0,0);

    public final double armAngle;
    public final double gripperAngle;
    public final double elevatorHeight;

    DemaciaRobotState(double armAngle, double gripperAngle, double elevatorHeight) {
        this.armAngle = armAngle;
        this.gripperAngle = gripperAngle;
        this.elevatorHeight = elevatorHeight;
    }

    public static DemaciaRobotState getStateBasedOnLevel(LEVEL level) {
        switch (level) {
            case L1:
                return L1;
            
            case L2:
                return L2;
            
            case L3:
                return L3;

            case L4:
                return L4;
            
            case FEEDER:
                return FEEDER;
        
            default:
                return IDLE;
        }
    }
}

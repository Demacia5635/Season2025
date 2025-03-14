package frc.robot.robot2;

public enum DemaciaRobotState {
    L1(-2.55, -1.8, 0),
    L2(1.8, -0.5, 0),
    L3(0.8, -1.3, 0),
    L4(0.6, -1.5, 0.64),
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
}

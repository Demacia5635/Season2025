package frc.robot.practice;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AllOffsets extends ParallelCommandGroup{

    public AllOffsets() {
        addCommands(
            new L3Left(),
            new L3Right(),
            new L2Left(),
            new L2Right(),
            new Feeder(),
            new AlgaeBottomLeft(),
            new AlgaeBottomRight(),
            new AlgaeTopLeft(),
            new AlgaeTopRight()
        );
    }
}
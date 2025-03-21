package frc.robot.practice;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AllOffsets extends ParallelCommandGroup{

    public AllOffsets() {
        addCommands(
            new L3Left().ignoringDisable(true),
            new L3Right().ignoringDisable(true),
            new L2Left().ignoringDisable(true),
            new L2Right().ignoringDisable(true),
            new Feeder().ignoringDisable(true),
            new AlgaeTop().ignoringDisable(true),
            new AlgaeBottom().ignoringDisable(true)
        );
    }
}
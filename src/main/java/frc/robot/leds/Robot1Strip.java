package frc.robot.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.leds.subsystems.LedStrip;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class Robot1Strip extends LedStrip {

    private final Arm arm;
    private final Gripper gripper;

    private final Timer grabTimer;
    private final Timer dropTimer;
    private final Timer autoPathTimer;
    private final Timer coralStationTimer;
    
    public Robot1Strip(Arm arm, Gripper gripper) {
        super("Robot 1 Strip", LedConstants.LENGTH, RobotContainer.ledManager);
        
        this.arm = arm;
        this.gripper = gripper;

        this.grabTimer = new Timer();
        this.dropTimer = new Timer();
        this.autoPathTimer = new Timer();
        this.coralStationTimer = new Timer();
    }

    public void setCoralStation() {
        coralStationTimer.start();
    }

    public void setAutoPath() {
        autoPathTimer.start();
    }

    public void setDrop() {
        dropTimer.start();
    }

    public void setGrab() {
        grabTimer.start();
    }

    @Override
    public void periodic() {


        setColor(Color.kWhite);

        if (arm.getState().equals(ARM_ANGLE_STATES.CORAL_STATION)) {
            setColor(Color.kYellow);
        }
        
        if (!grabTimer.hasElapsed(2) && grabTimer.get() != 0) {
            setBlink(Color.kYellow);
        } else if (grabTimer.hasElapsed(2)) {
            grabTimer.stop();
            grabTimer.reset();
        }

        if (!dropTimer.hasElapsed(2) && dropTimer.get() != 0) {
            setBlink(Color.kPurple);
        } else if (dropTimer.hasElapsed(2)) {
            dropTimer.stop();
            dropTimer.reset();
        }

        if (gripper.isCoral()) {
            setColor(Color.kPurple);
        }

        if (arm.getCurrentCommand() instanceof ArmCalibration) {
            setSolidGay();
        }
        
        if (!autoPathTimer.hasElapsed(1) && autoPathTimer.get() != 0) {
            setBlink(Color.kRed);
        } else if (autoPathTimer.hasElapsed(1)) {
            autoPathTimer.stop();
            autoPathTimer.reset();
        }

        if (!coralStationTimer.hasElapsed(2) && coralStationTimer.get() != 0) {
            setBlink(Color.kGreen);
        } else if (coralStationTimer.hasElapsed(2)) {
            coralStationTimer.stop();
            coralStationTimer.reset();
        }
    }
}

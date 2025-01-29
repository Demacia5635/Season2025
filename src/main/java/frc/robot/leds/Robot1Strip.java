package frc.robot.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.leds.subsystems.LedStrip;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;

public class Robot1Strip extends LedStrip {

    private final Arm arm;
    private final Gripper gripper;

    private final Timer grabTimer;
    private final Timer dropTimer;
    
    public Robot1Strip(Arm arm, Gripper gripper) {
        super("Robot 1 Strip", LedConstants.LENGTH, RobotContainer.ledManager);
        
        this.arm = arm;
        this.gripper = gripper;

        this.grabTimer = new Timer();
        this.dropTimer = new Timer();
    }

    @Override
    public void periodic() {
        switch (arm.getState()) {
            case CORAL_STATION:
                setColor(Color.kYellow);
                break;
            
            case L2_TOUCHING:
                setColor(Color.kDarkGreen);
                break;
            
            case L3_TOUCHING:
                setColor(Color.kDarkBlue);
                break;
            
            case STARTING:
                setColor(Color.kWhite);
                break;
        
            default:
                setSolidGay();
                break;
        }

        if (gripper.isCoral()) {
            setColor(Color.kPurple);
        }

        if (gripper.getCurrentCommand() instanceof Grab && !grabTimer.isRunning()) {
            grabTimer.start();
        }

        if (gripper.getCurrentCommand() instanceof Drop && !dropTimer.isRunning()) {
            dropTimer.start();
        }

        if (!grabTimer.hasElapsed(2) && grabTimer.get() != 0) {
            setBlink(Color.kRed);
        } else if (grabTimer.hasElapsed(2)){
            grabTimer.stop();
            grabTimer.reset();
        }

        if (!dropTimer.hasElapsed(2) && dropTimer.get() != 0) {
            setBlink(Color.kOrange);
        } else if (grabTimer.hasElapsed(2)){
            dropTimer.stop();
            dropTimer.reset();
        }
    }
    
}

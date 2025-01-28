package frc.robot.leds;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.leds.subsystems.LedStrip;

public class Robot1Strip extends LedStrip {

    XboxController controller;

    public Robot1Strip(CommandXboxController controller) {
        super("Robot 1 Strip", LedConstants.LENGTH, RobotContainer.ledManager);
        this.controller = controller.getHID();
    }

    @Override
    public void periodic() {
        if (controller.getAButton()) {
            setColor(Color.kBlue);
        } else if (controller.getBButton()) {
            setBlink(Color.kRed);
        } else if (controller.getLeftTriggerAxis() > 0.3) {
            setColor(new Color(0, controller.getLeftTriggerAxis(), 0));
        } else {
            setSolidGay();
        }
    }
    
}

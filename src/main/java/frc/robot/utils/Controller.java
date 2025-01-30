package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends CommandGenericHID{
    public enum ControllerType {
        kXbox,
        kPS5;
    }

    private final GenericHID m_hid;
    private final ControllerType controllerType;

    public Controller(int port, ControllerType controllerType) {
        super(port);
        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
        this.controllerType = controllerType;
        this.m_hid = new GenericHID(port);
    }

    public Trigger uptButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kY.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kTriangle.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger leftButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kX.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kSquare.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger downButton() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kA.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kCross.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger rightButton() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kB.value, CommandScheduler.getInstance().getDefaultButtonLoop());
        
            case kPS5:
                return button(PS5Controller.Button.kCircle.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger leftBumper() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kLeftBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kL1.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger rightBumper() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kRightBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kR1.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger leftStick() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kLeftStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kL3.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger rightStick() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kRightStick.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kL3.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger rightSetting() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kStart.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kOptions.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger leftSettings() {
        switch (controllerType) {
            case kXbox:
                return button(XboxController.Button.kBack.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kCreate.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }
}

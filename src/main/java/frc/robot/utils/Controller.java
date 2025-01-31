package frc.robot.utils;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends CommandGenericHID{

    public enum ControllerType {
        kXbox,
        kPS5;
    }

    private final ControllerType controllerType;

    public Controller(int port, ControllerType controllerType) {
        super(port);
        this.controllerType = controllerType;
        HAL.report(tResourceType.kResourceType_Controller, port + 1, 0, "Driver Controller");
    }

    public Trigger buttonUp() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kY.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kTriangle.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger buttonLeft() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kX.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kSquare.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger buttonDown() {
        switch(controllerType) {
            case kXbox:
                return button(XboxController.Button.kA.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            case kPS5:
                return button(PS5Controller.Button.kCross.value , CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger buttonRight() {
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

    public double getLeftX() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kLeftX.value);
            case kPS5:
                return getRawAxis(PS5Controller.Axis.kLeftX.value);
            default:
                return 0;
        }
    }

    public double getLeftY() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kLeftY.value);
            case kPS5:
                return getRawAxis(PS5Controller.Axis.kLeftY.value);
            default:
                return 0;
        }
    }

    public double getRightX() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kRightX.value);
            case kPS5:
                return getRawAxis(PS5Controller.Axis.kRightX.value);
            default:
                return 0;
        }
    }

    public double getRightY() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kRightY.value);
            case kPS5:
                return getRawAxis(PS5Controller.Axis.kRightY.value);
            default:
                return 0;
        }
    }

    public double getLeftTrigger() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kLeftTrigger.value);
            case kPS5:
                return (getRawAxis(PS5Controller.Axis.kL2.value) + 1) / 2;
            default:
                return 0;
        }
    }

    public double getRightTrigger() {
        switch (controllerType) {
            case kXbox:
                return getRawAxis(XboxController.Axis.kRightTrigger.value);
            case kPS5:
                return (getRawAxis(PS5Controller.Axis.kR2.value) + 1) / 2;
            default:
                return 0;
        }
    }

    public Trigger getLeftX(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftX.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kLeftX.value, threshold);
            default:
                return null;
        }
    }

    public Trigger getLeftY(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftY.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kLeftY.value, threshold);
            default:
                return null;
        }
    }
    public Trigger getRightX(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightX.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kRightX.value, threshold);
            default:
                return null;
        }
    }

    public Trigger getRightY(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightY.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kRightY.value, threshold);
            default:
                return null;
        }
    }

    public Trigger getLeftTrigger(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kLeftTrigger.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kL2.value, threshold * 2  - 1);
            default:
                return null;
        }
    }

    public Trigger getRightTrigger(double threshold) {
        switch (controllerType) {
            case kXbox:
                return axisGreaterThan(XboxController.Axis.kRightTrigger.value, threshold);
            case kPS5:
                return axisGreaterThan(PS5Controller.Axis.kR2.value, threshold * 2 - 1);
            default:
                return null;
        }
    }

    public Trigger getPS() {
        switch (controllerType) {
            case kXbox:
                LogManager.log("Xbox controller does not have PS button", AlertType.kError);
                return null;
            case kPS5:
                return button(PS5Controller.Button.kPS.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }

    public Trigger getTouchPad() {
        switch (controllerType) {
            case kXbox:
                LogManager.log("Xbox controller does not have touchpad", AlertType.kError);
                return null;        
            case kPS5:
                return button(PS5Controller.Button.kTouchpad.value, CommandScheduler.getInstance().getDefaultButtonLoop());
            default:
                return null;
        }
    }
}
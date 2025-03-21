package frc.robot.practice;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.utils.LogManager;

public class L2Right extends Command {

    double x;
    double y;

    double lastX;
    double lastY;

    public L2Right() {
        this.x = 0;
        this.y = 0;

        this.lastX = 0;
        this.lastY = 0;
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putData("Offsets/L2 Right", this);
        x = FieldTarget.l2Right.getX();
        y = FieldTarget.l2Right.getY();

        lastX = x;
        lastY = y;
    }

    @Override
    public void execute() {
        if (lastX != x || lastY != y) {
            FieldTarget.l2Right = new Translation2d(x, y);
            
            lastX = x;
            lastY = y;
        }
    }

    @Override
    public void end(boolean interrupted) {
        LogManager.log("Offset: L3Left x: " + x + "y: " + y);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("x", () -> x, value -> x = value);
        builder.addDoubleProperty("y", () -> y, value -> y = value);
    }
}
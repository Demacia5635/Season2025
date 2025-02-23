package frc.robot.practice;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.utils.LogManager;

public class AlgaeTopLeft extends Command {

    double x;
    double y;

    double lastX;
    double lastY;

    public AlgaeTopLeft() {
        this.x = 0;
        this.y = 0;

        this.lastX = 0;
        this.lastY = 0;
        
        SmartDashboard.putData("AlgaeTopLeft", this);
    }

    @Override
    public void initialize() {
        x = FieldTarget.topAlgeaLeftOffset.getX();
        y = FieldTarget.topAlgeaLeftOffset.getY();

        lastX = x;
        lastY = y;
    }

    @Override
    public void execute() {
        if (lastX != x || lastY != y) {
            FieldTarget.topAlgeaLeftOffset = new Translation2d(x, y);
            
            lastX = x;
            lastY = y;
        }
    }

    @Override
    public void end(boolean interrupted) {
        LogManager.log("Offset: Feeder x: " + x + "y: " + y);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("x", () -> x, value -> x = value);
        builder.addDoubleProperty("y", () -> y, value -> y = value);
    }
}
package frc.robot.practice;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.utils.LogManager;

public class PracticeOffsets extends Command{
    private double x;
    private double y;
    private OffsetType type;

    public enum OffsetType{
        L3_LEFT,
        L3_RIGHT,
        L2_LEFT,
        L2_RIGHT,
        FEEDER,
        ALGAE_BOTTOM_LEFT,
        ALGAE_BOTTOM_RIGHT,
        ALGAE_TOP_LEFT,
        ALGAE_TOP_RIGHT
    }

    public PracticeOffsets(OffsetType type) {
        this.x = 0;
        this.y = 0;
        this.type = type;
        SmartDashboard.putData(type.name(), this);
    }

    @Override
    public void initialize() {
        switch (type) {
            case L2_LEFT:
                x = FieldTarget.l2Left.getX();
                y = FieldTarget.l2Left.getY();
                break;

            case L2_RIGHT:
                x = FieldTarget.l2Right.getX();
                y = FieldTarget.l2Right.getY();
                break;
            
            case L3_LEFT:
                x = FieldTarget.l3Left.getX();
                y = FieldTarget.l3Left.getY();
                break;
            
            case L3_RIGHT:
                x = FieldTarget.l3Right.getX();
                y = FieldTarget.l3Right.getY();
                break;
            
            case FEEDER:
                x = FieldTarget.intakeOffset.getX();
                y = FieldTarget.intakeOffset.getY();
                break;
            
            case ALGAE_TOP_LEFT:
                x = FieldTarget.topAlgeaLeftOffset.getX();
                y = FieldTarget.topAlgeaLeftOffset.getY();
                break;
            
            case ALGAE_TOP_RIGHT:
                x = FieldTarget.topAlgeaRightOffset.getX();
                y = FieldTarget.topAlgeaRightOffset.getY();
                break;
            
            case ALGAE_BOTTOM_RIGHT:
                x = FieldTarget.bottomAlgeaRightOffset.getX();
                y = FieldTarget.bottomAlgeaRightOffset.getY();
                break;
            
            case ALGAE_BOTTOM_LEFT:
                x = FieldTarget.bottomAlgeaLeftOffset.getX();
                y = FieldTarget.bottomAlgeaLeftOffset.getY();
                break;

            default:
                break;
        }
    }

    @Override
    public void execute() {
        switch (type) {
            case L2_LEFT:
                FieldTarget.l2Left = new Translation2d(x, y);
                break;

            case L2_RIGHT:
                FieldTarget.l2Right = new Translation2d(x, y);
                break;

            case L3_LEFT:
                FieldTarget.l3Left = new Translation2d(x, y);
                break;

            case L3_RIGHT:
                FieldTarget.l3Right = new Translation2d(x, y);
                break;

            case FEEDER:
                FieldTarget.intakeOffset = new Translation2d(x, y);
                break;

            case ALGAE_TOP_LEFT:
                FieldTarget.topAlgeaLeftOffset = new Translation2d(x, y);
                break;
                
            case ALGAE_TOP_RIGHT:
                FieldTarget.topAlgeaRightOffset = new Translation2d(x, y);
                break;

            case ALGAE_BOTTOM_RIGHT:
                FieldTarget.bottomAlgeaRightOffset = new Translation2d(x, y);
                break;

            case ALGAE_BOTTOM_LEFT:
                FieldTarget.bottomAlgeaLeftOffset = new Translation2d(x, y);
                break;
            
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        LogManager.log("Offset: " + type.name() + " x: " + x + " y: " + y);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("x", ()-> x, value -> x = value);
        builder.addDoubleProperty("y", ()-> y, value -> y = value);
    }
}

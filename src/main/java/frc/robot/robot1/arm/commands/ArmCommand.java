// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.arm.constants.ArmConstants.ANGLES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.arm.utils.ArmUtils;
import frc.robot.utils.LogManager;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

public class ArmCommand extends Command {
  private Arm arm;
  private double testArmAngle;
  private double testGripperAngle;
  private Pair<Double, Double> wantedAngle;

  Pose2d currentPose2d;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm) {
    this.arm = arm;
    testArmAngle = arm.getArmAngle();
    testGripperAngle = arm.getGripperAngle();
    SmartDashboard.putData(this);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (arm.getState()) {
      case L2_CALC:
        wantedAngle = ArmUtils.calcAngles(currentPose2d.getTranslation().minus(FieldConstants.REEF).getNorm(), FieldConstants.L2_HEIGHT);
        break;

      case L3_CALC:
        wantedAngle = ArmUtils.calcAngles(currentPose2d.getTranslation().minus(FieldConstants.REEF).getNorm(), FieldConstants.L3_HEIGHT);
        break;

      case L2_TOUCHING:
        wantedAngle = ANGLES.L2;
        break;
      
      case L3_TOUCHING:
        wantedAngle = ANGLES.L3;
        break;

      case ALGAE_UNDER:
        wantedAngle = ANGLES.ALGAE_UNDER;
        break;

      case ALGAE_OVER:
        wantedAngle = ANGLES.ALGAE_OVER;
        break;

      case CORAL_STATION:
        wantedAngle = ANGLES.CORAL_STATION;
        break;

      case TESTING:
        wantedAngle = new Pair<Double,Double>(testArmAngle, testGripperAngle);
        break;

      case STARTING:
        wantedAngle = ANGLES.STARTING;
        break;

      case IDLE:
        wantedAngle = new Pair<Double,Double>(arm.getArmAngle(), arm.getGripperAngle());
        arm.stop();

      default:
        LogManager.log("Arm state is illegal", AlertType.kError);
        arm.setState(ARM_ANGLE_STATES.IDLE);
        wantedAngle = new Pair<Double,Double>(arm.getArmAngle(), arm.getGripperAngle());
        arm.stop();

    }
    arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Test Arm Angle", () -> testArmAngle, value -> testArmAngle = value);
    builder.addDoubleProperty("Test Gripper Angle", () -> testGripperAngle, value -> testGripperAngle = value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

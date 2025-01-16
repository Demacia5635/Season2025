// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.constants.ArmConstants.ArmAngleMotorConstants;
import frc.robot.robot1.arm.constants.ArmConstants.FieldConstants;
import frc.robot.robot1.arm.constants.ArmConstants.GripperAngleMotorConstants;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.arm.utils.ArmUtils;

public class ArmCommand extends Command {
  private Arm arm;
  private double testArmAngle;
  private double testGripperAngle;
  private Pair<Double, Double> wantedAngle;

  Pose2d currentPose2d;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm) {
    this.arm = arm;
    testArmAngle = ArmAngleMotorConstants.BASE_ANGLE;
    testGripperAngle = GripperAngleMotorConstants.BASE_ANGLE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (arm.state) {
      case L2, L3:
        wantedAngle = ArmUtils.calcAngles(currentPose2d.getTranslation().minus(FieldConstants.REEF).getNorm(), arm.state.TARGET_HEIGHT);
        arm.setMotionMagic(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case CORAL_STATION:
        wantedAngle = ArmUtils.calcAngles(currentPose2d.getTranslation().minus(FieldConstants.CORAL_STATION).getNorm(), arm.state.TARGET_HEIGHT);
        arm.setMotionMagic(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case TESTING:
        arm.setMotionMagic(testArmAngle, testGripperAngle);
        break;

      case IDLE:
        arm.stop();

      default:

    }

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

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
import frc.robot.vision.utils.VisionConstants;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

import java.util.function.Supplier;

/**
 * the main command of the arm
 * <br>
 * </br>
 * the command checks the arm state and based on that gives the arm the wanted
 * angles
 */
public class ArmCommand extends Command {

  /* the arm subsytem */
  private final Arm arm;
  /*
   * test arm angle for the testing state (take variables out of network tables)
   */
  private double testArmAngle;
  /*
   * test gripper angle for the testing state (take variables out of network
   * tables)
   */
  private double testGripperAngle;
  /* the wanted angles */
  private Pair<Double, Double> wantedAngle;

  /* current position later will be set from the chassis */
  // Supplier<Translation2d> currentPos;

  /**
   * creates a new arm command
   * <br>
   * </br>
   * this function also configure the test angles to the current angles, put the
   * command in the smart dashboard and add the arm to the requirments
   * 
   * @param arm the wanted arm
   */
  public ArmCommand(Arm arm) {
    this.arm = arm;

    testArmAngle = arm.getArmAngle();
    testGripperAngle = arm.getGripperAngle();

    // this.currentPos = currentPos;

    SmartDashboard.putData(this);
    addRequirements(arm);
  }

  /**
   * this function is called at the start of the command
   * <br>
   * </br>
   * the function does nothing
   */
  @Override
  public void initialize() {
  }

  /**
   * this function is called every cycle of the command
   * <br>
   * </br>
   * 
   * <pre>
   * the function checks the state of the arm and then choosing what angles the arm needs to be
   * L2_CALC -> the calculated angles to score at L2
   * L3_CALC -> the calculated angles to score at L3
   * L2_TOUCHING -> the constant angles to score at L2
   * L3_TOUCHING -> the constant angles to score at L3
   * ALGAE_UNDER -> the constant angles to be under the algae in the reef
   * ALGAE_OVER -> the constant angles to be over the algae in the reef
   * CORAL_STATION -> the constant angles to grab from the coral station
   * TESTING -> the angles from the network tables
   * STARTING -> the constant angles at the start of the arm
   * IDLE -> the current angles (no movement from the arm)
   * Default -> logs an error and put the state at idle
   * </pre>
   */
  @Override
  public void execute() {
    switch (arm.getState()) {
      // case CALC:
      //   wantedAngle = ArmUtils.calculateAngles(
      //     currentPos.get().minus(
      //       RobotContainer.scoringTarget.position.getApproachPoint(
      //         RobotContainer.scoringTarget.elementPosition == ELEMENT_POSITION.CORAL_LEFT 
      //         ? FullAlign.reelLeftReefOffset 
      //         : FullAlign.reelRightReefOffset).getTranslation()).getNorm()
      //         - 0.25
      //       , FieldConstants.L2_HEIGHT + 0.05
      //     );
      //   LogManager.log(wantedAngle);
      //   if (wantedAngle.getFirst().isNaN() || wantedAngle.getSecond().isNaN()) {
      //     wantedAngle = ANGLES.L2;
      //   }
      //   arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
      //   break;

      case L2:
        wantedAngle = ANGLES.L2;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case L3:
        wantedAngle = ANGLES.L3;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case ALGAE_BOTTOM:
        wantedAngle = ANGLES.ALGAE_BOTTOM;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case ALGAE_TOP:
        wantedAngle = ANGLES.ALGAE_TOP;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case CORAL_STATION:
        wantedAngle = ANGLES.CORAL_STATION;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case TESTING:
        wantedAngle = new Pair<Double, Double>(testArmAngle, testGripperAngle);
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case STARTING:
        wantedAngle = ANGLES.STARTING;
        arm.setPositionVoltage(wantedAngle.getFirst(), wantedAngle.getSecond());
        break;

      case IDLE:
        wantedAngle = new Pair<Double, Double>(arm.getArmAngle(), arm.getGripperAngle());
        arm.stop();
        break;

      default:
        // LogManager.log("Arm state is illegal", AlertType.kError);
        arm.setState(ARM_ANGLE_STATES.IDLE);
        wantedAngle = new Pair<Double, Double>(arm.getArmAngle(), arm.getGripperAngle());
        arm.stop();
    }

  }

  /**
   * the init sendable of the command
   * <br>
   * </br>
   * putting the test arm angle and test gripper angle inside
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Test Arm Angle", () -> testArmAngle, value -> testArmAngle = value);
    builder.addDoubleProperty("Test Gripper Angle", () -> testGripperAngle, value -> testGripperAngle = value);
  }

  /**
   * This function is called after the command had finished
   * <br>
   * </br>
   * the function stop the arm
   */
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  /**
   * the function that is called to check if the command have finished
   * <br>
   * </br
   * there is no condition so the command will go all the time after calibration
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

/**
 * The Arm Subsystem
 * <br>
 * </br>
 * The subsytem is containing two motors and two sensors:
 * <br>
 * </br>
 * one motor is to change the angle of the motor and the other is to change the
 * angle of the gripepr
 * <br>
 * </br>
 * one sensor is digital and is being use to calibrate the angle and the other
 * is absolute sensor to the gripper angle in consta to the arm
 * <br>
 * </br>
 * the subsytem is worked on by state based system, the state variable is
 * deciding what the angles the arm needs to be
 * <br>
 * </br>
 * 
 * @see frc.robot.robot1.arm.constants.ArmConstants
 * @see frc.robot.robot1.arm.commands.ArmCommand
 */
public class Arm extends SubsystemBase {

  /** The motor that change the angle of the arm */
  private final TalonMotor armAngleMotor;
  /** The motor that change the angle of the gripper */
  private final TalonMotor gripperAngleMotor;

  /** The digital sensor that help with calibrate the arm */
  private final DigitalInput armAngleLimit;
  /**
   * The absolute sensor that tells the gripper angle motor what is the real angle
   * of the gripper
   */
  private final DutyCycleEncoder gripperAngleAbsoluteSensor;

  /**
   * A variable that tells the arm if it was calibrated if it wasn't its will not
   * listen to go to angles
   */
  public boolean isCalibrated;

  /**
   * The state of the arm used in the
   * {@link frc.robot.robot1.arm.commands.ArmCommand} to tell the arm what angle
   * to go
   */
  public ARM_ANGLE_STATES state;

  /**
   * creates a new Arm, should only be one
   * <br>
   * </br>
   * configure the motors and the sensor and puts all the needed stuff at the
   * network tables
   */
  public Arm() {
    /* set the name of the subsystem */
    setName(NAME);

    /* configure the motors */
    armAngleMotor = new TalonMotor(ArmAngleMotorConstants.CONFIG);
    gripperAngleMotor = new TalonMotor(GripperAngleMotorConstants.CONFIG);

    /* configure the sensors */
    armAngleLimit = new DigitalInput(ArmAngleMotorConstants.LIMIT_SWITCH_CHANNEL);
    gripperAngleAbsoluteSensor = new DutyCycleEncoder(GripperAngleMotorConstants.ABSOLUTE_SENSOR_CHANNEL);

    /* set is calibrated to false at the start */
    isCalibrated = false;

    /* make the default state to idle */
    state = ARM_ANGLE_STATES.IDLE;

    /* add to network tables everything that needed */
    addNT();
  }

  /**
   * add to network tables all the variables
   */
  public void addNT() {
    /* add to log the important stuff */
    LogManager.addEntry(getName() + "/Arm Angle", this::getArmAngle);
    LogManager.addEntry(getName() + "/Gripper Angle", this::getGripperAngle);
    LogManager.addEntry(getName() + "/Gripper Angle Motor", this::getGripperAngleMotor);
    LogManager.addEntry(getName() + "/Arm Angle Limit Switch", () -> getArmAngleLimit() ? 1 : 0);
    LogManager.addEntry(getName() + "/IsReady", () -> isReady() ? 1 : 0);

    /* add to smart dashboard the widgets of the talon motor */
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME, armAngleMotor);
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME, gripperAngleMotor);

    /* add to smart dashboard the coast and brake of both motors */
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set brake",
        new InstantCommand(() -> armAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set coast",
        new InstantCommand(() -> armAngleNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set brake",
        new InstantCommand(() -> gripperAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set coast",
        new InstantCommand(() -> gripperAngleNeutralMode(false)).ignoringDisable(true));

    /* add the arm itself to the network tables */
    SmartDashboard.putData(this);
  }

  /**
   * set the arm to calibrated
   * <br>
   * </br>
   * used in the {@link frc.robot.robot1.arm.commands.ArmCalibration}
   */
  public void hadCalibrated() {
    isCalibrated = true;
  }

  /**
   * set the state of the arm
   * 
   * @param state the wanted state
   */
  public void setState(ARM_ANGLE_STATES state) {
    this.state = state;
  }

  /**
   * get the current state of the arm
   * 
   * @return the current state
   */
  public ARM_ANGLE_STATES getState() {
    return state;
  }

  /**
   * set the arm angle motor neutral mode
   * 
   * @param isBrake is the motor needs to be brake (true -> brake; false -> coast)
   */
  public void armAngleNeutralMode(boolean isBrake) {
    armAngleMotor.setNeutralMode(isBrake);
  }

  /**
   * set the gripper angle motor neutral mode
   * 
   * @param isBrake is the motor needs to be brake (true -> brake; false -> coast)
   */
  public void gripperAngleNeutralMode(boolean isBrake) {
    gripperAngleMotor.setNeutralMode(isBrake);
  }

  /**
   * set power to the arm angle motor
   * 
   * @param power the wanted power from -1 to 1
   */
  public void armAngleMotorSetPower(double power) {
    armAngleMotor.setDuty(power);
  }

  /**
   * set the power to the gripper angle motor
   * 
   * @param power the wanted power from -1 to 1
   */
  public void gripperAngleMotorSetPower(double power) {
    gripperAngleMotor.setDuty(power);
  }

  /**
   * set power to both motors
   * 
   * @param armAnglePower     the wanted power for the arm angle motor from -1 to
   *                          1
   * @param gripperAnglePower the wanted power for the gripper angle motor from -1
   *                          to 1
   */
  public void setPower(double armAnglePower, double gripperAnglePower) {
    armAngleMotorSetPower(armAnglePower);
    gripperAngleMotorSetPower(gripperAnglePower);
  }

  /**
   * set the arm angle motor to position voltage with the target angle
   * 
   * @param targetAngle the wanted angle in radians
   * @see if the arm did not calibrate the request will not go through
   * @see if the target angle is below back limit the target will automaticly be
   *      the back limit
   * @see if the target angle is above the forward limit the target will be
   *      automaticly be the forward limit
   */
  public void armAngleMotorSetPositionVoltage(double targetAngle) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    if (targetAngle < ArmAngleMotorConstants.BACK_LIMIT) {
      targetAngle = ArmAngleMotorConstants.BACK_LIMIT;
    }
    if (targetAngle > ArmAngleMotorConstants.FWD_LIMIT) {
      targetAngle = ArmAngleMotorConstants.FWD_LIMIT;
    }

    armAngleMotor.setPositionVoltage(targetAngle);
  }

  /**
   * set the gripper angle motor to position voltage with the target angle
   * 
   * @param targetAngle the wanted angle in radians
   * @see if the arm did not calibrate the request will not go through
   * @see if the target angle is below back limit the target will automaticly be
   *      the back limit
   * @see if the target angle is above the forward limit the target will be
   *      automaticly be the forward limit
   * @see if the arm angle is below a specific angle the gripepr will always want
   *      to go to the back limit
   */
  public void gripperAngleMotorSetPositionVoltage(double angle) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    if (angle < GripperAngleMotorConstants.BACK_LIMIT) {
      angle = GripperAngleMotorConstants.BACK_LIMIT;
    }
    if (angle > GripperAngleMotorConstants.FWD_LIMIT) {
      angle = GripperAngleMotorConstants.FWD_LIMIT;
    }

    if (armAngleMotor.getCurrentClosedLoopSP() <= GripperAngleStarting.WHEN_MOVING_GRIPPER) {
      angle = GripperAngleStarting.ANGLE_TO_GRIPPER;
    }

    gripperAngleMotor.setPositionVoltage(angle);
  }

  /**
   * set position voltage to both motors
   * 
   * @param armAngle     the wanted angle in radians for the arm angle motor
   * @param gripperAngle the wanted angle in radians for the gripper angle motor
   */
  public void setPositionVoltage(double armAngle, double gripperAngle) {
    armAngleMotorSetPositionVoltage(armAngle);
    gripperAngleMotorSetPositionVoltage(gripperAngle);
  }

  /**
   * set the angle motor position
   * 
   * @param angle the new value the arm angle motor position will be
   */
  public void armAngleSetPosition(double angle) {
    armAngleMotor.setPosition(angle);
  }

  /**
   * stop both motors
   */
  public void stop() {
    armAngleMotor.stopMotor();
    gripperAngleMotor.stopMotor();
  }

  /**
   * get is the motors closed loop error are less than the max errors
   * 
   * @return is the motors at the right angles
   */
  public boolean isReady() {
    return armAngleMotor.getCurrentClosedLoopError() <= MaxErrors.ARM_ANGLE_ERROR
        && gripperAngleMotor.getCurrentClosedLoopError() <= MaxErrors.GRIPPER_ANGLE_ERROR;
  }

  /**
   * get the arm angle
   * 
   * @return the arm angle motor position, position in radians
   */
  public double getArmAngle() {
    return armAngleMotor.getCurrentPosition();
  }

  /**
   * @deprecated use the getGripperAngle function instead to get the gripper angle
   *             from the sensor
   *             get the gripper angle from the motor instead of the absolute
   *             sensor
   * @return the gripper angle motor position, position in radians
   */
  public double getGripperAngleMotor() {
    return gripperAngleMotor.getCurrentPosition();
  }

  /**
   * get the gripper angle absolute sensor
   * 
   * @return the gripper angle in radians
   */
  public double getGripperAngle() {
    return (gripperAngleAbsoluteSensor.get() * 2 * Math.PI) - GripperAngleMotorConstants.ENCODER_BASE_ANGLE;
  }

  /**
   * get the arm angle digital input
   * 
   * @return if the digital input is closed
   */
  public boolean getArmAngleLimit() {
    return !armAngleLimit.get();
  }

  /**
   * the init sendable of the command
   * this is for all the stuff that can not be in the log
   * 
   * @param builder the builder of the subsystem
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

  }

  /**
   * This function runs every cycle
   * <br>
   * </br>
   * the function check if the absolute gripper angle sensor is connected
   * <br>
   * </br>
   * the function set the gripper angle motor position to the absolute sensor
   */
  @Override
  public void periodic() {
    /* check if the gripper angle absolute sensor is connected */
    if (!gripperAngleAbsoluteSensor.isConnected()) {
      LogManager.log("Gripper Angle Encoder is not connected", AlertType.kError);
    }

    /* set the gripper angle motor position to the gripper angle absolute sensor */
    gripperAngleMotor.setPosition(getGripperAngle());
  }
}

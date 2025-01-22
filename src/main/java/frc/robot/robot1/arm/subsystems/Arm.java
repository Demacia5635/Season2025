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

public class Arm extends SubsystemBase {

  private TalonMotor armAngleMotor;
  private TalonMotor gripperAngleMotor;

  private DigitalInput armAngleLimit;
  private DigitalInput gripperAngleLimit;
  private DutyCycleEncoder gripperAngleEncoder;

  public boolean isCalibrated;
  
  public ARM_ANGLE_STATES state;
  private boolean isReady;

  public Arm() {
    setName(NAME);

    armAngleMotor = new TalonMotor(ArmAngleMotorConstants.CONFIG);
    gripperAngleMotor = new TalonMotor(GripperAngleMotorConstants.CONFIG);

    armAngleLimit = new DigitalInput(ArmAngleMotorConstants.LIMIT_SWITCH_CHANNEL);
    gripperAngleLimit = new DigitalInput(GripperAngleMotorConstants.LIMIT_SWITCH_CHANNEL);

    gripperAngleEncoder = new DutyCycleEncoder(GripperAngleMotorConstants.ENCODER_CHANNEL);

    isCalibrated = false;

    state = ARM_ANGLE_STATES.IDLE;
    isReady = true;

    gripperAngleMotor.setPosition(getGripperAngle());

    addNT();
  }

  public void addNT() {
    LogManager.addEntry(getName() + "/Arm Angle", this::getArmAngle);
    LogManager.addEntry(getName() + "/Gripper Angle", this::getGripperAngle);
    LogManager.addEntry(getName() + "/Gripper Angle Motor", this::getGripperAngleMotor);
    LogManager.addEntry(getName() + "/Arm Angle Limit Switch", () -> getArmAngleLimit() ? 1 : 0);
    LogManager.addEntry(getName() + "/Gripper Angle Limit Switch", () -> getGripperAngleLimit() ? 1 : 0);
    LogManager.addEntry(getName() + "/IsReady", ()-> isReady() ? 1 : 0);

    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME, armAngleMotor);
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME, gripperAngleMotor);

    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set brake", new InstantCommand(()-> armAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set coast", new InstantCommand(()-> armAngleNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set brake", new InstantCommand(()-> gripperAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set coast", new InstantCommand(()-> gripperAngleNeutralMode(false)).ignoringDisable(true));

    SmartDashboard.putData(this);
  }

  public void setState(ARM_ANGLE_STATES state) {
    this.state = state;
  }

  public ARM_ANGLE_STATES getState() {
    return state;
  }

  public void armAngleNeutralMode(boolean isBrake) {
    armAngleMotor.setNeutralMode(isBrake);
  }

  public void gripperAngleNeutralMode(boolean isBrake) {
    gripperAngleMotor.setNeutralMode(isBrake);
  }

  public void armAngleMotorSetPower(double power) {
    armAngleMotor.setDuty(power);
  }

  public void gripperAngleMotorSetPower(double power) {
    gripperAngleMotor.setDuty(power);
  }

  public void setPower(double armAnglePower, double gripperAnglePower) {
    armAngleMotorSetPower(armAnglePower);
    gripperAngleMotorSetPower(gripperAnglePower);
  }

  public void armAngleMotorSetMotionMagic(double angle) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    if (angle < ArmAngleMotorConstants.BACK_LIMIT) {
      angle = ArmAngleMotorConstants.BACK_LIMIT;
    }
    if (angle > ArmAngleMotorConstants.FWD_LIMIT) {
      angle = ArmAngleMotorConstants.FWD_LIMIT;
    }

    armAngleMotor.setMotionMagic(angle);
  }

  public void gripperAngleMotorSetMotionMagic(double angle) {
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

    gripperAngleMotor.setMotionMagic(angle);
  }

  public void setMotionMagic(double armAngle, double gripperAngle) {
    armAngleMotorSetMotionMagic(armAngle);
    gripperAngleMotorSetMotionMagic(gripperAngle);
  }

  public void gripperAngleSetVelocity(double velocity) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    if (getGripperAngle() < GripperAngleMotorConstants.BACK_LIMIT) {
      velocity = 0;
    }

    if (getGripperAngle() > GripperAngleMotorConstants.FWD_LIMIT) {
      velocity = 0;
    }
    
    gripperAngleMotor.setVelocity(velocity);
  }

  public void stop() {
    armAngleMotor.stopMotor();
    gripperAngleMotor.stopMotor();
  }

  public void armAngleSetPosition(double angle) {
    armAngleMotor.setPosition(angle);
  }

  private void checkIfIsReady() {
    isReady = armAngleMotor.getCurrentClosedLoopError() <= MaxErrors.ARM_ANGLE_ERROR
    && gripperAngleMotor.getCurrentClosedLoopError() <= MaxErrors.GRIPPER_ANGLE_ERROR;
  }

  public boolean isReady() {
    return isReady;
  }

  public double getArmAngle() {
    return armAngleMotor.getCurrentPosition();
  }

  public double getGripperAngleMotor() {
    return gripperAngleMotor.getCurrentPosition();
  }

  public double getGripperAngle() {
    return (gripperAngleEncoder.get() * 2 * Math.PI) + GripperAngleMotorConstants.ENCODER_BASE_ANGLE;
  }

  public boolean getArmAngleLimit() {
    return !armAngleLimit.get();
  }

  public boolean getGripperAngleLimit() {
    return !gripperAngleLimit.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

  }

  @Override
  public void periodic() {
    if (!gripperAngleEncoder.isConnected()) {
      LogManager.log("Gripper Angle Encoder is not connected", AlertType.kError);
    }

    checkIfIsReady();
  }
}

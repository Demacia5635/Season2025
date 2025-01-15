// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.subsystems;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

public class Arm extends SubsystemBase {

  private TalonMotor armAngleMotor;
  private TalonMotor intakeAngleMotor;

  private DigitalInput armAngleLimit;
  private DigitalInput intakeAngleLimit;

  public boolean isCalibrated;

  public Arm() {
    setName(NAME);

    armAngleMotor = new TalonMotor(ArmAngleMotorConstants.CONFIG);
    intakeAngleMotor = new TalonMotor(IntakeAngleMotorConstants.CONFIG);
    
    armAngleLimit = new DigitalInput(ArmAngleMotorConstants.LIMIT_SWITCH_CHANNEL);
    intakeAngleLimit = new DigitalInput(IntakeAngleMotorConstants.LIMIT_SWITCH_CHANNEL);

    isCalibrated = false;
    
    SmartDashboard.putData(ArmAngleMotorConstants.NAME, armAngleMotor);
    SmartDashboard.putData(IntakeAngleMotorConstants.NAME, intakeAngleMotor);

    addLog();
    SmartDashboard.putData(this);
  }

  public void addLog() {
    LogManager.addEntry(getName() + "/Arm Angle", this::getArmAngle);
    LogManager.addEntry(getName() + "/Intake Angle", this::getIntakeAngle);
    LogManager.addEntry(getName() + "/Arm Angle Limit Switch", () -> getArmAngleLimit() ? 1 : 0);
    LogManager.addEntry(getName() + "/Intake Angle Limit Switch", () -> getIntakeAngleLimit() ? 1 : 0);

  }

  public void armAngleMotorSetPower(double power) {
    armAngleMotor.setDuty(power);
  }

  public void intakeAngleMotorSetPower(double power) {
    intakeAngleMotor.setDuty(power);
  }

  public void setPower(double armAnglePower, double intakeAnglePower) {
    armAngleMotorSetPower(armAnglePower);
    intakeAngleMotorSetPower(intakeAnglePower);
  }

  public void armAngleMotorSetMotionMagic(double angle) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    armAngleMotor.setMotionMagic(angle);
  }

  public void intakeAngleMotorSetMotionMagic(double angle) {
    if (!isCalibrated) {
      LogManager.log("Can not move motor before calibration", AlertType.kError);
      return;
    }

    intakeAngleMotor.setMotionMagic(angle);
  }

  public void setMotionMagic(double armAngle, double intakeAngle) {
    armAngleMotorSetMotionMagic(armAngle);
    intakeAngleMotorSetMotionMagic(intakeAngle);
  }

  public void stop() {
    armAngleMotor.stopMotor();
    intakeAngleMotor.stopMotor();
  }

  public void armAngleSetPosition(double angle) {
    armAngleMotor.setPosition(angle);
  }

  public void intakeAngleSetPosition(double angle) {
    intakeAngleMotor.setPosition(angle);
  }

  public double getArmAngle() {
    return armAngleMotor.getCurrentPosition();
  }

  public double getIntakeAngle() {
    return intakeAngleMotor.getCurrentPosition();
  }

  public boolean getArmAngleLimit() {
    return armAngleLimit.get();
  }

  public boolean getIntakeAngleLimit() {
    return intakeAngleLimit.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

  }

  @Override
  public void periodic() {

  }
}

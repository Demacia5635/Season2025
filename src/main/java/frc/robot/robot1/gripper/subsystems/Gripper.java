// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LogManager;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.robot1.gripper.constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
  private final TalonSRX motor;
  private final AnalogInput sensor;

  public Gripper() {
    setName(NAME);

    motor = new TalonSRX(MotorConstants.MOTOR_ID);
    motor.setInverted(MotorConstants.INVERT ? InvertType.InvertMotorOutput : InvertType.None);
    motor.setNeutralMode(MotorConstants.START_NEUTRAL_MODE ? NeutralMode.Brake : NeutralMode.Coast);

    sensor = new AnalogInput(SensorConstants.SENSOR_CHANNEL);

    addNT();
  }

  private void addNT() {
    LogManager.addEntry(getName() + "/get Sensor", ()-> getSensor());
    LogManager.addEntry(getName() + "/Is Coral", ()-> isCoral() ? 1 : 0);
    
    LogManager.addEntry(getName() + "/Motor" + "/Velocity", motor::getSelectedSensorVelocity);
    
    SmartDashboard.putData(getName() + "/Motor" + "/set Brake", new InstantCommand(()-> setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/Motor" + "/set Coast", new InstantCommand(()-> setNeutralMode(false)).ignoringDisable(true));

    SmartDashboard.putData(this);
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public void stop() {
    motor.neutralOutput();
  }

  public void setNeutralMode(boolean isBrake) {
    motor.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public double getSensor() {
    return sensor.getVoltage();
  }

  public boolean isCoral() {
    return getSensor() < SensorConstants.CORAL_IN_SENSOR;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  @Override
  public void periodic() {
  }
}

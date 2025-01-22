// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LogManager;

import static frc.robot.robot1.gripper.constants.GripperConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Gripper extends SubsystemBase {
  private final TalonSRX motor;
  private final DigitalInput sensor;

  public Gripper() {
    setName(NAME);

    motor = new TalonSRX(MotorConstants.MOTOR_ID);
    motor.setInverted(MotorConstants.INVERT ? InvertType.InvertMotorOutput : InvertType.None);
    motor.setNeutralMode(MotorConstants.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);

    sensor = new DigitalInput(SensorConstants.SENSOR_CHANNEL);

    SmartDashboard.putData(this);
    addNT();
  }

  private void addNT() {
    LogManager.addEntry(getName() + "/Is Sensor", ()-> getSensor() ? 1 : 0);
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public void stop() {
    setPower(0);
  }

  public boolean getSensor() {
    return sensor.get();
  }

  @Override
  public void periodic() {
  }
}

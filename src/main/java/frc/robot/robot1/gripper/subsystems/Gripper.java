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

/**
 * The gripper subsytem.
 * <br>
 * </br>
 * the gripper contains one motor and one sensor.
 * <br>
 * </br>
 * the motor is a red line motor that powers the gripper
 * <br>
 * </br>
 * the sensor is an analog sensor that at the end of the gripper telling when a
 * coral is inside the gripper
 */
public class Gripper extends SubsystemBase {
  /** The motor of the gripper */
  private final TalonSRX motor;

  private final AnalogInput upFrontSensor;
  private final AnalogInput upBackSensor;
  /** The sensor that tells if a coral is inside */
  private final AnalogInput downSensor;

  /**
   * creates a new gripper, supposed to be only one.
   * <br>
   * </br>
   * the function confiure the motor and sensor and then send to network tables
   * staff
   */
  public Gripper() {
    /* set the name of the subsystem */
    setName(NAME);

    /* congiure the motor with invert and neutral mode */
    motor = new TalonSRX(MotorConstants.MOTOR_ID);
    motor.setInverted(MotorConstants.INVERT ? InvertType.InvertMotorOutput : InvertType.None);
    motor.setNeutralMode(MotorConstants.START_NEUTRAL_MODE ? NeutralMode.Brake : NeutralMode.Coast);

    /* configure the sensors */
    upFrontSensor = new AnalogInput(SensorConstants.UP_FRONT_SENSOR_CHANNEL);
    upBackSensor = new AnalogInput(SensorConstants.UP_BACK_SENSOR_CHANNEL);
    downSensor = new AnalogInput(SensorConstants.DOWN_SENSOR_CHANNEL);

    /* send to network tables staff */
    addNT();
  }

  /**
   * add to network tables staff
   */
  private void addNT() {
    /* put the sensors */
    LogManager.addEntry(getName() + "/get up front sensor", this::getUpFrontSensor);
    LogManager.addEntry(getName() + "/get up back sensor", this::getUpBackSensor);
    LogManager.addEntry(getName() + "/get down sensor", this::getDownSensor);
    LogManager.addEntry(getName() + "/Is Coral", this::isCoral);

    /* put function to put the motor at brake and coast */
    SmartDashboard.putData(getName() + "/Motor" + "/set Brake",
        new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/Motor" + "/set Coast",
        new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));

    /* put the gripper itself in the smart dashboard */
    SmartDashboard.putData(this);
  }

  /**
   * set power to the motor
   * 
   * @param power the wanted power from -1 to 1
   */
  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  /**
   * stop the motor
   */
  public void stop() {
    motor.neutralOutput();
  }

  /**
   * set the neutral mode of the motor
   * 
   * @param isBrake is the motor needs to be in brake (true -> brake; false ->
   *                coast)
   */
  public void setNeutralMode(boolean isBrake) {
    motor.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public double getUpFrontSensor() {
    return upFrontSensor.getAverageVoltage();
  }

  public double getUpBackSensor() {
    return upBackSensor.getAverageVoltage();
  }

  /**
   * get the sensor voltage
   * 
   * @return the sensor voltage
   */
  public double getDownSensor() {
    return downSensor.getVoltage();
  }

  public boolean isCoralUpSensor() {
    return getUpFrontSensor() < SensorConstants.CORAL_IN_SENSOR 
    || getUpBackSensor() < SensorConstants.CORAL_IN_SENSOR;
  }

  public boolean isCoralDownSensor() {
    return getDownSensor() < SensorConstants.CORAL_IN_SENSOR;
  }

  /**
   * get is a coral inside the gripper.
   * <br>
   * </br>
   * if the sensor voltage is below a specific voltage
   * 
   * @return is a coral inside the gripper
   */
  public boolean isCoral() {
    return isCoralUpSensor() && isCoralDownSensor();
  }

  /**
   * the init sendable of the gripper
   * 
   * @param builder the sendable builder of the function
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

  /**
   * the function that runs every cycle of the subsystem
   * <br>
   * </br>
   * does nothing
   */
  @Override
  public void periodic() {
  }
}

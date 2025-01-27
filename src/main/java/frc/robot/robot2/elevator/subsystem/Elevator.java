// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator.subsystem;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.TalonMotor;
import frc.robot.robot2.elevator.ElevatorConstants;
import frc.robot.utils.LogManager;

public class Elevator extends SubsystemBase {

  TalonMotor motor;
  DigitalInput topLimitSwitch;
  DigitalInput bottomLimitSwitch;

  boolean isCalibrated;

  public Elevator() {
    setName(ElevatorConstants.NAME);

    this.motor = new TalonMotor(ElevatorConstants.motorConfig);

    this.topLimitSwitch = new DigitalInput(ElevatorConstants.ElevatorLimits.TOP_SWITCH_ID);
    this.bottomLimitSwitch = new DigitalInput(ElevatorConstants.ElevatorLimits.BOTTOM_SWITCH_ID);
    this.isCalibrated = false;

    addNT();
  }
  private void addNT() {
    LogManager.addEntry(getName() + "/Top Limit Switch", () -> hasReachedTop() ? 1 : 0);
    LogManager.addEntry(getName() + "/Bottom Limit Switch", () -> hasReachedBottom() ? 1 : 0);

    SmartDashboard.putData(getName() + "/Motor", motor);

    SmartDashboard.putData(this);
  }

  public void calibrated() {
    isCalibrated = true;
  }


  private boolean hasLimitSwitch(DigitalInput input){
    return !input.get();
  }

  public boolean hasReachedBottom(){
    return hasLimitSwitch(bottomLimitSwitch);
  }

  public boolean hasReachedTop(){
    return hasLimitSwitch(topLimitSwitch);
  }
  
  /*
   * position in meters
   */
  public void setPositionVoltage(double position){
    if (isCalibrated) {
      LogManager.log("Elevator has not calibrated", AlertType.kError);
      return;
    }

    motor.setPositionVoltage(position);
  }

  public void setHeight(double height) {
    motor.setPosition(height);
  }

  public void setPower(double power) {
    motor.setDuty(power);
  }

  public void stop() {
    motor.stopMotor();
  }

}

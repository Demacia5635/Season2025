// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator.subsystem;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.TalonMotor;
import frc.robot.robot2.elevator.ElevatorConstants;
import frc.robot.utils.LogManager;

public class Elevator extends SubsystemBase {

  TalonMotor motor;
  public Elevator() {
    setName(ElevatorConstants.NAME);

    this.motor = new TalonMotor(ElevatorConstants.motorConfig);

    addNT();
  }
  private void addNT() {

    SmartDashboard.putData(getName() + "/Motor", motor);

    SmartDashboard.putData(this);
  }

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

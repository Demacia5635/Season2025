// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.elevator.subsystem;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.TalonMotor;
import frc.robot.utils.LogManager.LogEntry;
import frc.robot.robot2.elevator.ElevatorConstants;
import frc.robot.robot2.elevator.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.robot2.elevator.ElevatorConstants.ElevatorLimits;
import frc.robot.robot2.elevator.ElevatorConstants.ElevatorMotorConstants;
import static frc.robot.robot2.elevator.ElevatorConstants.ElevatorMotorConstants.*;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.utils.LogManager;

public class Elevator extends SubsystemBase {

  TalonMotor motor;
  DigitalInput topLimitSwitch;
  DigitalInput bottomLimitSwitch;

  ELEVATOR_STATE state;

  double error;

  boolean isCalibrated;
  double voltest = 0;

  public Elevator() {
    setName(ElevatorConstants.NAME);

    this.motor = new TalonMotor(ElevatorMotorConstants.motorConfig);

    this.topLimitSwitch = new DigitalInput(ElevatorLimits.TOP_SWITCH_ID);
    this.bottomLimitSwitch = new DigitalInput(ElevatorLimits.BOTTOM_SWITCH_ID);

    this.state = ELEVATOR_STATE.IDLE;

    this.isCalibrated = false;

    addNT();
  }
  
  private void addNT() {
    LogManager.addEntry(getName() + "/Top Limit Switch", () -> hasReachedTop() ? 1 : 0);
    LogManager.addEntry(getName() + "/Bottom Limit Switch", () -> hasReachedBottom() ? 1 : 0);

    SmartDashboard.putData(getName() + "/Motor", motor);

    SendableChooser<ELEVATOR_STATE> stateChooser = new SendableChooser<ELEVATOR_STATE>();
    stateChooser.addOption("L4", ELEVATOR_STATE.L4);
    stateChooser.addOption("L3", ELEVATOR_STATE.L3);
    stateChooser.addOption("L2", ELEVATOR_STATE.L2);
    stateChooser.addOption("Coral Station", ELEVATOR_STATE.CORAL_STATION);
    stateChooser.addOption("Starting", ELEVATOR_STATE.STARTING);
    stateChooser.addOption("Testing", ELEVATOR_STATE.TESTING);
    stateChooser.addOption("Idle", ELEVATOR_STATE.IDLE);
    stateChooser.onChange(state -> this.state = state);
    SmartDashboard.putData(getName() + "/State Chooser", stateChooser);

    motor.configPidFf(0);
    motor.configMotionMagic();

    SmartDashboard.putData(getName() + "/Motor" + "/Set Brake", new InstantCommand(()-> motor.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/Motor" + "/Set Coast", new InstantCommand(()-> motor.setNeutralMode(false)).ignoringDisable(true));

    SmartDashboard.putData("set voltage", new RunCommand(() -> motor.setVoltage(voltest)));

    SmartDashboard.putData(this);
  }

  public void setNeutalMode(boolean isBrake) {
    motor.setNeutralMode(isBrake);
  }

  public void calibrated() {
    isCalibrated = true;
  }

  public ELEVATOR_STATE getState() {
    return state;
  }

  public void setState(ELEVATOR_STATE state) {
    this.state = state;
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
  
  public void setMagicMotion(double position){
    if (!isCalibrated) {
      LogManager.log("Elevator has not calibrated", AlertType.kError);
      return;
    }
    if (position > ElevatorLimits.TOP_LIMIT_POSITION) {
      position = ElevatorLimits.TOP_LIMIT_POSITION;
    }
    if (position < ElevatorLimits.BOTTOM_LIMIT_POSITION) {
      position = ElevatorLimits.BOTTOM_LIMIT_POSITION;
    }
    motor.setMotionMagic(position, kG);
  }

  /*
   * position in meters
   */
  public void setPositionVoltage(double position){
    if (!isCalibrated) {
      LogManager.log("Elevator has not calibrated", AlertType.kError);
      return;
    }

    if (position > ElevatorLimits.TOP_LIMIT_POSITION) {
      position = ElevatorLimits.TOP_LIMIT_POSITION;
    }
    if (position < ElevatorLimits.BOTTOM_LIMIT_POSITION) {
      position = ElevatorLimits.BOTTOM_LIMIT_POSITION;
    }

    motor.setPositionVoltage(position, ElevatorMotorConstants.kG); //kg
  }

  /*
   * position in meters
   */
  public void setPosition(double position){
    if (!isCalibrated) {
      LogManager.log("Elevator has not calibrated", AlertType.kError);
      return;
    }

    if (position > ElevatorLimits.TOP_LIMIT_POSITION) {
      position = ElevatorLimits.TOP_LIMIT_POSITION;
    }
    if (position < ElevatorLimits.BOTTOM_LIMIT_POSITION) {
      position = ElevatorLimits.BOTTOM_LIMIT_POSITION;
    }

    double curretPotion = motor.getCurrentPosition();
    error = position - curretPotion;
    if(Math.abs(error) < ALLOWED_ERROR) {
      motor.setDuty(0);
      return;
    }
    double tgtVel = 0;
    double direction = error > 0? 1 : -1;
    double curretVel = motor.getCurrentVelocity();
    if(Math.abs(error) < DEACCEL_DISTANCE) {
      tgtVel = Math.sqrt(2 * error * direction * ACC);
    } else {
      if(direction > 0) {
        tgtVel = Math.min(MAX_VEL, curretVel + DELATA_VEL);
      } else {
        tgtVel = Math.max(-MAX_VEL, curretVel - DELATA_VEL);
      }
    }
    double a = (tgtVel - curretVel) / DELATA_TIME;
    double p = kS*direction + curretVel*kV + a*kA + kG;
    motor.setDuty(p/12.0);
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

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("test vol", ()-> voltest, (double vel)-> voltest = vel);
  }

  @Override
  public void periodic() {
  }
}

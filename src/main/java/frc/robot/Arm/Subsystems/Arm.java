// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Arm.ArmConstants.MOTORS_CONFIGS.*;

import frc.robot.Arm.ArmConstants.STATE;
import frc.robot.utils.*;

public class Arm extends SubsystemBase {
  private TalonMotor motor1;
  private TalonMotor motor2;
  public STATE state;
  public boolean isL2Default;
  public Arm() {
    motor1 = new TalonMotor(new TalonConfig(
      MOTOR1_ID, CANBUS, MOTOR1_NAME)
      .withPID(MOTOR1_KP, MOTOR1_KI, MOTOR1_KD, MOTOR1_KS, MOTOR1_KV, MOTOR1_KA, MOTOR1_KG)
      .withMotionMagic(MOTOR1_MAX_VELOCITY,MOTOR1_MAX_Acceleration,MOTOR1_MAX_JERK)
      .withBrake(IS_MOTOR1_BRAKE)
      .withInvert(IS_MOTOR1_INVERT)
      .withRadiansMotor()
      .withMotorRatio(MOTOR1_GEAR_RATIO));
    motor2 = new TalonMotor(new TalonConfig(
      MOTOR2_ID, CANBUS, MOTOR2_NAME)
      .withPID(MOTOR2_KP, MOTOR2_KI, MOTOR2_KD, MOTOR2_KS, MOTOR2_KV, MOTOR2_KA, MOTOR2_KG)
      .withMotionMagic(MOTOR2_MAX_VELOCITY,MOTOR2_MAX_Acceleration,MOTOR2_MAX_JERK)
      .withBrake(IS_MOTOR2_BRAKE)
      .withInvert(IS_MOTOR2_INVERT)
      .withRadiansMotor()
      .withMotorRatio(MOTOR2_GEAR_RATIO));
      state = STATE.DEFAULT;
      isL2Default = true;
  }

  @Override
  public void periodic() {

  }

  public void setMotor1Power(double power){
    motor1.setDuty(power);
  }

  public void setMotor1Position(double angle){
    motor1.setMotionMagic(angle);
  }

  public void setMotor2Power(double power){
    motor2.setDuty(power);
  }

  public void setMotor2Position(double angle){
    motor2.setMotionMagic(angle);
  }

  public double getMotor1Angle(){
    return motor1.getCurrentPosition();
  }

  public double getMotor2Angle(){
    return motor2.getCurrentPosition();
  }

  public void setAngle(){
    motor1.setPosition(0);
    motor2.setPosition(0);
  }
}

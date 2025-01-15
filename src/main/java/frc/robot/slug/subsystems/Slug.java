// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.slug.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.slug.SlugConstants;
import frc.robot.utils.TalonMotor;

public class Slug extends SubsystemBase {
  private final TalonMotor motor;
  private final DigitalInput sensor;

  public Slug() {
    motor = new TalonMotor(SlugConstants.MOTOR_CONFIG);
    sensor = new DigitalInput(SlugConstants.SENSOR_CHANNEL);
  }

  public void setPower(double power) {
    motor.set(power);
  }

  public void stop() {
    setPower(0);
  }

  public Command getFeedCommand() {
    return new InstantCommand(() -> setPower(SlugConstants.FEED_POWER), this)
      .raceWith(new WaitUntilCommand(sensor::get))
      .andThen(() -> stop(), this);
  }

  public Command getExcreteCommand() {
    return new InstantCommand(() -> setPower(SlugConstants.EXCRETE_POWER), this).withTimeout(SlugConstants.EXCRETE_DURATION)
      .andThen(() -> stop(), this);
  }

  @Override
  public void periodic() {
  }
}

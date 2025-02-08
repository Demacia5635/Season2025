// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.climb.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot1.climb.ClimebConstants;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

public class Climb extends SubsystemBase {
  private TalonMotor cllimbMotor;
  private DigitalInput AngleLimit;
  public Climb() {
    cllimbMotor = new TalonMotor(ClimebConstants.MOTOR_CONFIG);
    AngleLimit = new DigitalInput(ClimebConstants.LIMIT_SWITCH_CHANNEL);

    LogManager.addEntry(getName() + "/climeb Angle", this::getArmAngle);
    LogManager.addEntry(getName() + "/climeb is on Limit Switch", this::getLimit);

  }

  public void setClimbPower(double power){
    cllimbMotor.set(power);
  }

  public void stopClimb(){
    cllimbMotor.set(0);;
  }

  public void breakClimb(){
    cllimbMotor.setNeutralMode(true);;
  }
  
  public void coastClimb(){
    cllimbMotor.setNeutralMode(false);;
  }

  public double getArmAngle(){
    return cllimbMotor.getCurrentPosition();
  }

  public boolean getLimit() {
    return !AngleLimit.get();
  }

  public void setAngle(double angle){
    cllimbMotor.setPosition(angle);
  }
  
  @Override
  public void periodic() {
    
  }
}

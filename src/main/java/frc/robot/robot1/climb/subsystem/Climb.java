// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.climb.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot1.climb.ClimebConstants;
import frc.robot.utils.TalonMotor;

public class Climb extends SubsystemBase {
  private TalonMotor cllimbMotor;
  public Climb() {
    cllimbMotor = new TalonMotor(ClimebConstants.MOTOR_CONFIG);
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

  public double getCurrent(){
    return cllimbMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isStall(){
    return getCurrent() > ClimebConstants.ClimbConstants.STALL_CURRENT;
  }

  @Override
  public void periodic() {
    
  }
}

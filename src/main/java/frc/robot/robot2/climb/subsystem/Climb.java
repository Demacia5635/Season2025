// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.climb.subsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot2.climb.ClimebConstants;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

public class Climb extends SubsystemBase {
  // private TalonMotor climbMotor;
  private DigitalInput angleLimit;
  public Climb() {
    setName(ClimebConstants.NAME);

    // climbMotor = new TalonMotor(ClimebConstants.MOTOR_CONFIG);
    angleLimit = new DigitalInput(ClimebConstants.LIMIT_SWITCH_CHANNEL);

    // SmartDashboard.putData(getName() + "/" + ClimebConstants.NAME + " Motor", climbMotor);

    // LogManager.addEntry(getName() + "/climeb Angle", this::getArmAngle);
    LogManager.addEntry(getName() + "/climeb is on Limit Switch", this::getLimit);

    // SmartDashboard.putData("Climb", this);
  }

  public void setClimbPower(double power){
    // climbMotor.set(power);
  }

  public void stopClimb(){
    // climbMotor.stopMotor();
  }

  public void breakClimb(){
    // climbMotor.setNeutralMode(true);
  }
  
  public void coastClimb(){
    // climbMotor.setNeutralMode(false);
  }

  public double getArmAngle(){
    return 0;
    // return climbMotor.getCurrentPosition();
  }

  public boolean getLimit() {
    return !angleLimit.get();
  }

  public void setAngle(double angle){
    // climbMotor.setPosition(angle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
  
  @Override
  public void periodic() {
    
  }
}

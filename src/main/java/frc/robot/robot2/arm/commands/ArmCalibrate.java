// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot2.arm.constants.ArmConstants.CalibrationConstants;
import frc.robot.robot2.arm.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCalibrate extends Command {
  private Arm arm;
  //private final Timer timer;

  /** Creates a new ArmCalibrate. */
  public ArmCalibrate(Arm arm) {
    //timer = new Timer();
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timer.start();
    arm.armAngleMotorSetPower(CalibrationConstants.ARM_ANGLE_START_POW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //timer.hasElapsed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armAngleMotorSetPower(0);

    /*
     * sets the arm to calibration mode
     */
    if(!interrupted){
      arm.hasCalibrated();
    }
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isLimit();
  }
}

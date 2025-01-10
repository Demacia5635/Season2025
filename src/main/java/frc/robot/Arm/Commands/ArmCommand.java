// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm.ArmConstants.STATE;
import frc.robot.Arm.Subsystems.Arm;
import static frc.robot.Arm.ArmConstants.ANGLES.*;
import static frc.robot.Arm.ArmUtils.functions.*;

public class ArmCommand extends Command {
  private Arm arm;
  private double angle1;
  private double angle2;
  private double distance;
  public STATE state;
  private boolean isAngle1Finished;
  private boolean isAngle2Finished;
  private boolean isFinished;
  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm) {
    this.arm = arm;
    angle1 = DEFULT_ANGLE1;
    angle2 = DEFULT_ANGLE2;
    state = arm.state;
    isAngle1Finished = false;
    isAngle2Finished = false;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case L2:
        angle1 = L2_ANGLE1;
        angle2 = L2_ANGLE2;
        //angle1 = getAnglesL2(distance)[0];
        //angle2 = getAnglesL2(distance)[1];
        break;

      case L3:
        angle1 = L3_ANGLE1;
        angle2 = L3_ANGLE2;
        //angle1 = getAnglesL3(distance)[0];
        //angle2 = getAnglesL3(distance)[1];
        break;

      case TESTING:
        angle1 = TESTING_ANGLE1;
        angle2 = TESTING_ANGLE2;
        break;

      case DEFULT:
        angle1 = DEFULT_ANGLE1;
        angle2 = DEFULT_ANGLE2;
        break;
    }
    arm.setMotor1Position(angle1);
    arm.setMotor2Position(angle2);
    isAngle1Finished = Math.abs(arm.getMotor1Angle()-angle1)<ANGLE1_RANGE;
    isAngle2Finished = Math.abs(arm.getMotor2Angle()-angle2)<ANGLE2_RANGE;
    isFinished = isAngle1Finished && isAngle2Finished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setMotor1Power(0);
    arm.setMotor2Power(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

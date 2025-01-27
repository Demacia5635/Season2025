// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.gripper.constants.GripperConstants;
import frc.robot.robot1.gripper.constants.GripperConstants.FixConstants;
import frc.robot.robot1.gripper.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FixGrab extends Command {
    private Gripper gripper;
    private boolean first = true;
  public FixGrab(Gripper gripper) {
    this.gripper = gripper;
    addRequirements(gripper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(gripper.isCoral() && first){
      gripper.setPower(-FixConstants.FIX_POWER);
    }
    if(!gripper.isCoral()){
      first =  false;
      gripper.setPower(FixConstants.FIX_POWER);
    }


  }

  @Override
  public void end(boolean interrupted) {
    gripper.stop();
  }

  @Override
  public boolean isFinished() {
    return gripper.isCoral() && !first;
  }
}

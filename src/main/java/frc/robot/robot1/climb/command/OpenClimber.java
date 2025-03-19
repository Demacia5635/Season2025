// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.climb.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.climb.ClimebConstants;
import frc.robot.robot1.climb.subsystem.Climb;
import frc.robot.utils.CommandController;
import frc.robot.utils.Elastic;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenClimber extends Command {

  private Climb climb;

  public OpenClimber(CommandController controller, Climb climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    Elastic.selectTab("End Game");
  }


  @Override
  public void execute() {
    climb.setClimbPower(ClimebConstants.ClimbConstants.prepareClimbPower);
    RobotContainer.arm.setState(ARM_ANGLE_STATES.CLIMB);
  }

  @Override
  public void end(boolean interrupted) {
    climb.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return climb.getLimit();
  }
}

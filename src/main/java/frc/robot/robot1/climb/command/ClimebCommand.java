// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.climb.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot1.climb.ClimebConstants;
import frc.robot.robot1.climb.subsystem.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimebCommand extends Command {
  private Climb climb;
  private Timer timer;
  public ClimebCommand(Climb climb) {
    this.climb = climb;
    timer = new Timer();
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  @Override
  public void execute() {
    if(timer.get() < ClimebConstants.ClimbConstants.prepareClimbTime){
      climb.setClimbPower(ClimebConstants.ClimbConstants.prepareClimbPower);
    }
    else{
      climb.setClimbPower(ClimebConstants.ClimbConstants.CLIMB_POWER);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climb.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return climb.isStall();
  }
}

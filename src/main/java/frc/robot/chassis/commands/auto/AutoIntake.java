
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends SequentialCommandGroup {
  Command goNearIntake;
  Command alignToIntake;

  BooleanSupplier isSeeTag;
  Chassis chassis;
  
  double distanceFromTag = 2;
  

  public AutoIntake(Chassis chassis, Arm arm, Gripper gripper, boolean isRight) {
    this.goNearIntake = new RunCommand(
      ()->chassis.goTo(
      AutoUtils.fieldElements.get(isRight ?  FIELD_POSITION.FEEDER_RIGHT : FIELD_POSITION.FEEDER_LEFT),  0.1), chassis)
        .alongWith(new InstantCommand(()->arm.setState(ARM_ANGLE_STATES.CORAL_STATION)));  


    this.alignToIntake = new RunCommand(()->chassis.goTo(
      AutoUtils.fieldElementsPractical.get(isRight ?  FIELD_POSITION.FEEDER_RIGHT : FIELD_POSITION.FEEDER_LEFT),  0.05), chassis)
      .withDeadline(new Grab(gripper));    
    

    isSeeTag = ()->chassis.isSeeTag(isRight ? 2 : 1, 1, distanceFromTag);

    addCommands(goNearIntake.until(isSeeTag), alignToIntake);
  }
}

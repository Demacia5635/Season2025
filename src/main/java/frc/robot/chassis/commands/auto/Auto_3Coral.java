// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Path.Utils.*;

import static frc.robot.chassis.commands.auto.AutoUtils.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/*public class Auto_3Coral extends Command {
  Command cmd;
  PathPoint dumyPoint = new PathPoint(0, 0, Rotation2d.fromDegrees(0), 0);
  PathPoint scoring1 = new PathPoint(11.88, 2.31, Rotation2d.fromDegrees(50), 0.1);
  PathPoint anchor = new PathPoint(new Translation2d(15.5, 1.2), Rotation2d.fromDegrees(0), 0.3);
  PathPoint feeder = new PathPoint(new Translation2d(16.77, 0.766), Rotation2d.fromDegrees(-50));
 

  public Auto_3Coral() {
  }

  @Override
  public void initialize() {
    cmd = goToMultiple(new PathPoint[]{dumyPoint, scoring1} ,3.5, Rotation2d.fromDegrees(50), false, false);

    
      
    cmd = cmd.andThen(new AlignToTag(chassis, true, true, true));

    cmd = cmd.andThen(goToMultiple(
      new PathPoint[]{dummyPoint, new PathPoint(new Translation2d(12.1, 2.4), Rotation2d.fromDegrees(0))},
       3.5, Rotation2d.fromDegrees(50), true, true));
     
       
      cmd = cmd.andThen(goToMultiple(
        new PathPoint[]{dummyPoint, new PathPoint(new Translation2d(16, 1.24), new Rotation2d(), 0)},
         3, feeder.getRotation(), false, false));
      
      cmd = cmd.andThen(new AlignToTag(chassis, false, false, true));


      cmd = cmd.andThen(new WaitCommand(0.5));

      cmd = cmd.andThen(goToLine(new Pose2d(14.54, 2.3, Rotation2d.fromDegrees(135)), 
        new Pose2d(16, 1.24, feeder.getRotation()), 1.5).withTimeout(1));

      
      cmd = cmd.andThen(new AlignToTag(chassis, true, true, true));

      cmd = cmd.andThen(goToMultiple(
        new PathPoint[]{dummyPoint, new PathPoint(new Translation2d(16, 1.24), new Rotation2d(), 0)},
         3, feeder.getRotation(), false, false));

      
      cmd = cmd.andThen(new AlignToTag(chassis, false, false, true));    
      
      cmd = cmd.andThen(goToLine(new Pose2d(13.54, 2.3, Rotation2d.fromDegrees(135)), 
        new Pose2d(15.7, 1.24, feeder.getRotation()), 1.5).withTimeout(1));
      
      cmd = cmd.andThen(new WaitCommand(0.3));

      cmd = cmd.andThen(new AlignToTag(chassis, false, true, true));
      

      
    
    
    
    

    cmd.schedule();
  }


}*/

package frc.robot.chassis.commands.auto;

import static frc.robot.chassis.commands.auto.AutoUtils.chassis;
import static frc.robot.chassis.commands.auto.AutoUtils.fieldElements;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_ELEMENTS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToPlace extends Command {


  private PathPoint dummyPoint = new PathPoint(new Translation2d(), new Rotation2d());
  FIELD_ELEMENTS element;
  double maxVel;
  public goToPlace(FIELD_ELEMENTS element, double maxVel) {
    this.element = element;
    this.maxVel = maxVel;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AutoUtils.goToMultiple(new PathPoint[]
    {dummyPoint, AutoUtils.fieldElements.get(element)}, maxVel, fieldElements.get(element).getRotation(),
     false).andThen(new AlignToTag(chassis, !AutoUtils.isLeft(element),
      element != FIELD_ELEMENTS.FEEDER_LEFT || element != FIELD_ELEMENTS.FEEDER_RIGHT)).schedule();

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

package frc.robot.chassis.commands.auto;

import static frc.robot.chassis.commands.auto.AutoUtils.chassis;
import static frc.robot.chassis.commands.auto.AutoUtils.fieldElements;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.chassis.commands.auto.AutoUtils.ELEMENT;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.commands.auto.AutoUtils.LEVEL;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.gripper.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToPlace extends Command {


  private PathPoint dummyPoint = new PathPoint(new Translation2d(), new Rotation2d());
  Gripper gripper;
  Arm arm;
  FIELD_POSITION position;
  ELEMENT element;
  LEVEL level;
  double maxVel;
  boolean isFeeder;
  Command cmd;
  public goToPlace(Arm arm, Gripper gripper, FIELD_POSITION position, ELEMENT element, LEVEL level, double maxVel) {
    this.arm = arm;
    this.gripper = gripper;
    this.position = position;
    this.element = element;
    this.maxVel = maxVel;
    this.level = level;
    this.isFeeder = position == FIELD_POSITION.FEEDER_LEFT || position == FIELD_POSITION.FEEDER_RIGHT;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*cmd = AutoUtils.goToMultiple(new PathPoint[]
    {dummyPoint, AutoUtils.fieldElements.get(position)}, maxVel, fieldElements.get(position).getRotation(),
     false, true, position, new AlignToTag(chassis,  element != ELEMENT.CORAL_LEFT,
     position != FIELD_POSITION.FEEDER_LEFT || position != FIELD_POSITION.FEEDER_RIGHT, false)) // .alongWith(new InstantCommand(()->RobotContainer.arm.setState(ARM_ANGLE_STATES.STARTING)))
    .andThen(new AutoAlign(chassis, arm, gripper, position,
      element, level).alongWith(new InstantCommand(()-> {if (position == FIELD_POSITION.FEEDER_LEFT) chassis.setRobotRelVelocities(new ChassisSpeeds(1, 0, 0));}, chassis)));
    cmd.schedule();*/

  }

  @Override
  public boolean isFinished() {
    return !cmd.isScheduled();
  }
}

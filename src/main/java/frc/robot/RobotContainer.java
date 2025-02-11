// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.commands.auto.L2AlgaeL3;
import frc.robot.chassis.commands.auto.RemoveAlgae;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.commands.ArmCommand;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.climb.command.JoyClimeb;
import frc.robot.robot1.climb.command.OpenClimber;
import frc.robot.robot1.climb.subsystem.Climb;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.Grab;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.leds.Robot1Strip;
import frc.robot.leds.subsystems.LedManager;
import frc.robot.utils.CommandController;
import frc.robot.utils.LogManager;
import frc.robot.utils.CommandController.ControllerType;
import frc.robot.vision.utils.VisionConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{

  public static RobotContainer robotContainer;
  public static LedManager ledManager;
  public static CommandController driverController;
  public static CommandController operatorController;
  public static boolean isRed = true;
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;

  public static Chassis chassis;  
  public static Arm arm;
  public static Gripper gripper;
  public static Climb climb;
  public static Robot1Strip robot1Strip;
  
  public static FieldTarget scoringTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);
  public static FieldTarget feedingTarget = new FieldTarget(POSITION.FEEDER_LEFT, ELEMENT_POSITION.FEEDER, LEVEL.FEEDER);

  Timer timer;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    robotContainer = this;
    new AutoUtils();
    new LogManager();
    ledManager = new LedManager();
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kPS5);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);
    timer = new Timer();
    LogManager.addEntry("Timer",()-> 15-timer.get());

    SmartDashboard.putData("Reef", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Reef");
        builder.addDoubleProperty("Position", ()-> scoringTarget.position.ordinal(), index-> {
          if (isFeeding(index, 0)) {
            // feedingTarget.position = POSITION.values()[(int)index];
          } else {
            scoringTarget.position = POSITION.values()[(int)index];
          }});
        builder.addDoubleProperty("Element Position", ()-> scoringTarget.elementPosition.ordinal(), index-> {
          if (isFeeding(index, 1)) {
            // feedingTarget.elementPosition = ELEMENT_POSITION.values()[(int)index];
          } else {
            scoringTarget.elementPosition = ELEMENT_POSITION.values()[(int)index];
          }
        });
        builder.addDoubleProperty("Level", ()-> scoringTarget.level.ordinal(), index-> {
          if (isFeeding(index, 2)) {
            // feedingTarget.level = LEVEL.values()[(int)index];
          } else {
            scoringTarget.level = LEVEL.values()[(int)index];
          }
        });
      }

      boolean isFeeding(double index, int element) {
        switch (element) {
          case 0:
            return index == 6 || index == 7;
          case 1:
            return index == 3;
          case 2:
            return index == 2;
          default:
            return false;
        }
      }
    });

    configureSubsytems();
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * This function start all the subsytems.
   * Put here all the subsystems you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureSubsytems() {
    chassis = new Chassis();
    arm = new Arm();
    gripper = new Gripper();
    climb = new Climb();
    robot1Strip = new Robot1Strip(chassis, arm, gripper);
  }

  /**
   * This function set all the default commands to the subsystems.
   * set all the default commands of the subsytems.
   * This function is called at the robot container constractor
   */
  private void configureDefaultCommands() {
    chassis.setDefaultCommand(new Drive(chassis, driverController));
    climb.setDefaultCommand(new JoyClimeb(operatorController, climb));
    arm.setDefaultCommand(new ArmCommand(arm));
  }


  private void configureBindings() {
    driverController.getLeftStickMove().onTrue(new Drive(chassis, driverController));
    driverController.povLeft().onTrue(new FollowTrajectory(chassis, new FieldTarget(POSITION.F, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L2)));

    driverController.rightButton().onTrue(new InstantCommand(()-> Drive.invertPrecisionMode()));
    driverController.downButton().onTrue(new FollowTrajectory(chassis, false));
    driverController.leftButton().onTrue(new FollowTrajectory(chassis, true));
    driverController.upButton().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.STARTING)).ignoringDisable(true));
    
    driverController.leftBumper().onTrue(new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
    }, chassis, arm, gripper).ignoringDisable(true));
    driverController.rightBumper().onTrue(new Command() {
      public void end(boolean interrupted) {
        if (gripper.isCoral()) {
          new Drop(gripper).schedule(); 
        } else {
          new Grab(gripper).schedule();
        }
      }
      public boolean isFinished() {
        return true;
      }
    });
    
    driverController.povUp().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.L3)).ignoringDisable(true));
    driverController.povDown().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.L2)).ignoringDisable(true));
    
    driverController.leftSettings().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.CORAL_STATION)).ignoringDisable(true));
    driverController.rightSetting().onTrue(new Command() {
      POSITION closestTagToChassis() {
        POSITION closestReef = null;
        double closestReefToRobot = Double.MAX_VALUE;
        for (int i = 0; i < 6; i++) {
          double norm = chassis.getPose().getTranslation().minus(VisionConstants.O_TO_TAG[POSITION.values()[i].getId()]).getNorm();
          if (norm < closestReefToRobot) {
            closestReefToRobot = norm;
            closestReef = POSITION.values()[i];
          }
        }
        return closestReef;
      }

      public void initialize() {
        POSITION closestReef = closestTagToChassis();
        if (closestReef != null) {
          scoringTarget.position = closestReef;
        }
      };

      public void end(boolean interrupted) {
        if (!interrupted) {
          new FollowTrajectory(chassis, true).schedule();
        }
      };

      public boolean isFinished() {
        return true;
      };
    });
    
    operatorController.upButton().onTrue(new InstantCommand(()-> chassis.setYaw(Rotation2d.kZero)).ignoringDisable(true));
    operatorController.rightButton().onTrue(new InstantCommand((robot1Strip::setCoralStation)).ignoringDisable(true));
    operatorController.downButton().onTrue(new RunCommand(()-> gripper.setPower(-1), gripper));
    operatorController.leftButton().onTrue(new ArmCalibration(arm));

    operatorController.leftBumper().onTrue(new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      arm.setState(ARM_ANGLE_STATES.IDLE);
    }, chassis, arm, gripper).ignoringDisable(true));

    operatorController.rightBumper().onTrue(new OpenClimber(climb));

    operatorController.povUp().onTrue(new InstantCommand(robot1Strip::setManualOrAuto));
    operatorController.povRight().onTrue(new InstantCommand(gripper::stop, gripper).ignoringDisable(true));
    operatorController.povDown().onTrue(new InstantCommand(chassis::stop, chassis).ignoringDisable(true));
    operatorController.povLeft().onTrue(new InstantCommand(()-> {arm.stop(); arm.setState(ARM_ANGLE_STATES.IDLE);}, arm).ignoringDisable(true));
  }

  public static boolean isRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean isComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::isComp, RobotContainer::setIsComp);
  }

  /**
   * This command is schedules at the start of teleop.
   * Look in {@link Robot} for more details.
   * @return the ommand that start at the start at enable
   */
  public Command getEnableInitCommand() {
    return new ArmCalibration(arm);
  }

  /**
   * This command is schedules at the start of disable.
   * Put here all the stop functions of all the subsytems and then add them to the requirments
   * This insures that the motors do not keep their last control mode earlier and moves uncontrollably.
   * Look in {@link Robot} for more details.
   * @return the command that runs at disable
   */
  public Command getDisableInitCommand() {
    Command initDisableCommand = new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      climb.stopClimb();
      timer.stop();
    }, chassis, arm, gripper, climb
    ).ignoringDisable(true);
    initDisableCommand.setName("Init Disable Command");
    return initDisableCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    timer.reset();
    timer.start();
    return (new ArmCalibration(arm).andThen(new ArmCommand(arm))).alongWith(new L2AlgaeL3(chassis, hasRemovedFromLog));  
  }
}

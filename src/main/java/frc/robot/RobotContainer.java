// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PowerDistributionConstants;
import frc.robot.Path.Trajectory.ChangeReefToClosest;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.commands.auto.Test;
import frc.robot.chassis.commands.auto.AlgaeL3;
import frc.robot.chassis.commands.auto.AlgaeL3L3;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.commands.ArmCommand;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.climb.command.JoyClimeb;
import frc.robot.robot1.climb.command.OpenClimber;
import frc.robot.robot1.climb.subsystem.Climb;
import frc.robot.robot1.gripper.commands.Drop;
import frc.robot.robot1.gripper.commands.GrabOrDrop;
import frc.robot.robot1.gripper.commands.GripperDrive;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.leds.Robot1Strip;
import frc.robot.leds.subsystems.LedManager;
import frc.robot.utils.CommandController;
import frc.robot.utils.LogManager;
import frc.robot.utils.CommandController.ControllerType;

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
  public static boolean isRed;
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;

  public static Chassis chassis;  
  public static Arm arm;
  public static Gripper gripper;
  public static Climb climb;
  public static Robot1Strip robot1Strip;
  
  public static FieldTarget scoringTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

  private final Timer timer;
  private final SendableChooser<AutoMode> autoChooser;
  private enum AutoMode {
    LEFT, MIDDLE, RIGHT
  }
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    robotContainer = this;
    new LogManager();
    ledManager = new LedManager();
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);
    timer = new Timer();
    LogManager.addEntry("Timer",()-> 15-timer.get());
    SmartDashboard.putData("Reef", ReefWidget.getInstance());
    SmartDashboard.putData("PDH", new PowerDistribution(PowerDistributionConstants.POWER_DISTRIBUTION_ID, PowerDistributionConstants.MODULE_TYPE));
    
    configureSubsytems();
    new AutoUtils();
    configureDefaultCommands();
    configureBindings();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("left", AutoMode.LEFT);
    autoChooser.addOption("middle", AutoMode.MIDDLE);
    autoChooser.addOption("right", AutoMode.RIGHT);
    SmartDashboard.putData("AutoChooser", autoChooser);

  }

  /**
   * This function start all the subsytems.
   * Put here all the subsystems you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureSubsytems() {
    Ultrasonic.setAutomaticMode(true);
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
    arm.setDefaultCommand(new ArmCommand(arm));
  }


  private void configureBindings() {
    driverController.getLeftStickMove().onTrue(new Drive(chassis, driverController));
    driverController.getRightStickkMove().onTrue(new JoyClimeb(driverController, climb));
    driverController.rightStick().onTrue(new OpenClimber(driverController, climb));

    driverController.rightButton().onTrue(new InstantCommand(()-> Drive.invertPrecisionMode()));
    driverController.downButton().onTrue(new FollowTrajectory(chassis, false));
    driverController.leftButton().onTrue(new FollowTrajectory(chassis, true));
    driverController.upButton().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.STARTING)).ignoringDisable(true));
    
    driverController.leftBumper().onTrue(new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      climb.stopClimb();
    }, chassis, arm, gripper, climb).ignoringDisable(true));
    driverController.rightBumper().onTrue(new GrabOrDrop(gripper));
    
    driverController.povUp().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.L3)).ignoringDisable(true));
    driverController.povDown().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.L2)).ignoringDisable(true));

    driverController.leftSettings().onTrue(new InstantCommand(()-> arm.setState(ARM_ANGLE_STATES.CORAL_STATION)).ignoringDisable(true));
    driverController.rightSetting().onTrue(new ChangeReefToClosest(chassis));

    operatorController.getRightStickkMove().onTrue(new JoyClimeb(operatorController, climb));
    
    operatorController.upButton().onTrue(new InstantCommand(()-> chassis.setYaw(Rotation2d.kZero)).ignoringDisable(true));
    operatorController.rightButton().onTrue(new InstantCommand((robot1Strip::setCoralStation)).ignoringDisable(true));
    operatorController.downButton().whileTrue(new GripperDrive(gripper, operatorController));
    operatorController.leftButton().onTrue(new ArmCalibration(arm));

    operatorController.leftBumper().onTrue(new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      arm.setState(ARM_ANGLE_STATES.IDLE);
      climb.stopClimb();
    }, chassis, arm, gripper, climb).ignoringDisable(true));

    operatorController.povUp().onTrue(new InstantCommand(climb::stopClimb ,climb).ignoringDisable(true));
    operatorController.povRight().onTrue(new InstantCommand(gripper::stop, gripper).ignoringDisable(true));
    operatorController.povDown().onTrue(new InstantCommand(chassis::stop, chassis).ignoringDisable(true));
    operatorController.povLeft().onTrue(new InstantCommand(()-> {arm.stop(); arm.setState(ARM_ANGLE_STATES.IDLE);}, arm).ignoringDisable(true));

    operatorController.rightSetting().onTrue(new InstantCommand(robot1Strip::setManualOrAuto).ignoringDisable(true));
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
    return new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      climb.stopClimb();
      timer.stop();
    }, chassis, arm, gripper, climb
    ).withName("initDisableCommand").ignoringDisable(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    timer.reset();
    timer.start();
    return (new ArmCalibration(arm).andThen(new Test().alongWith(new ArmCommand(arm))));
    // switch (autoChooser.getSelected()) {
    //   case LEFT:
    //     return new ArmCalibration(arm).andThen(new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, isRed, false)));

    //   case MIDDLE:
    //     return new ArmCalibration(arm).andThen(new ArmCommand(arm).alongWith(new AlgaeL3(chassis)));
      
    //   case RIGHT: 
    //     return new ArmCalibration(arm).andThen(new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, isRed, true)));
    
    //   default:
    //     return new ArmCalibration(arm);
    // }
  }
}

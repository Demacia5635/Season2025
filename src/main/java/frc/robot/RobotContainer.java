// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PowerDistributionConstants;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.FEEDER_SIDE;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot2.elevator.commands.ElevatorCalibration;
import frc.robot.robot2.elevator.commands.ElevatorCommand;
import frc.robot.robot2.elevator.subsystem.Elevator;
import frc.robot.robot2.gripper.commands.GrabOrDrop;
import frc.robot.robot2.gripper.subsystems.Gripper;
import frc.robot.robot2.DemaciaRobotState;
import frc.robot.robot2.arm.commands.ArmCommand;
import frc.robot.robot2.arm.subsystems.Arm;
import frc.robot.robot2.climb.command.ClimbUntilSensor;
import frc.robot.robot2.climb.command.JoyClimeb;
import frc.robot.robot2.climb.command.OpenClimber;
import frc.robot.robot2.climb.subsystem.Climb;
import frc.robot.utils.CommandController;
import frc.robot.utils.Elastic;
import frc.robot.utils.LogManager;
import frc.robot.utils.CommandController.ControllerType;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{

  public static RobotContainer robotContainer;
  public static CommandController driverController;
  public static CommandController operatorController;
  public static boolean isRed;
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;

  public static Chassis chassis;  
  public static Arm arm;
  public static Gripper gripper;
  public static Climb climb;
  public static Elevator elevator;
  // public static robot2Strip robot2Strip;
  
  public static FieldTarget scoringTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);

  public static DemaciaRobotState robotState = DemaciaRobotState.IDLE;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    robotContainer = this;
    new LogManager();
    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kPS5);
    operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);

    
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);
    LogManager.addEntry("Timer", DriverStation::getMatchTime);
    SmartDashboard.putData("Reef", ReefWidget.getInstance());  
    // SmartDashboard.putData("PDH", new PowerDistribution(PowerDistributionConstants.POWER_DISTRIBUTION_ID, PowerDistributionConstants.MODULE_TYPE));
    // SmartDashboard.putData("Offsets/Practice", new AllOffsets());
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Start Robot Code", ""));
    // SmartDashboard.putData("pracice", new AllOffsets());

    robotState = DemaciaRobotState.IDLE;
    SendableChooser<DemaciaRobotState> robotStateChooser = new SendableChooser<>();
    for (DemaciaRobotState state : DemaciaRobotState.values()) {
      robotStateChooser.addOption(state.name(), state);
    }
    robotStateChooser.setDefaultOption("IDLE2", DemaciaRobotState.IDLE);
    robotStateChooser.onChange(state -> robotState = state);
    SmartDashboard.putData("Robot State Chooser", robotStateChooser);
    
    configureSubsytems();
    // new AutoUtils();
    configureDefaultCommands();
    configureBindings();

    currentFeederSide = FEEDER_SIDE.MIDDLE;
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
    elevator = new Elevator();
    // robot2Strip = new robot2Strip(chassis, arm, gripper);
  }

  /**
   * This function set all the default commands to the subsystems.
   * set all the default commands of the subsytems.
   * This function is called at the robot container constractor
   */
  private void configureDefaultCommands() {
    chassis.setDefaultCommand(new Drive(chassis, driverController));
    elevator.setDefaultCommand(new ElevatorCommand(elevator));
    arm.setDefaultCommand(new ArmCommand(arm));
    //arm.setDefaultCommand(new ArmDrive(arm, driverController));
  }

  public static FEEDER_SIDE currentFeederSide;

  private void configureBindings() {
    driverController.getLeftStickMove().onTrue(new Drive(chassis, driverController));
    driverController.getRightStickkMove().onTrue(new JoyClimeb(driverController, climb));
    FieldTarget testTarget = new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_RIGHT, LEVEL.L4);
    FieldTarget test2 = new FieldTarget(POSITION.B, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L4);
    driverController.leftButton().onTrue(new FollowTrajectory(chassis, testTarget));
    driverController.downButton().onTrue(new FollowTrajectory(chassis, test2));

    driverController.rightBumper().onTrue(new GrabOrDrop(gripper));
    
    operatorController.rightBumper().onTrue(new ClimbUntilSensor(climb));
    operatorController.leftBumper().onTrue(new InstantCommand(()-> {
      chassis.stop();
      arm.stop();
      gripper.stop();
      climb.stopClimb();
    }, chassis, arm, gripper, climb).ignoringDisable(true));

    // operatorController.povUp().onTrue(new InstantCommand(climb::stopClimb ,climb).ignoringDisable(true));
    // operatorController.povRight().onTrue(new InstantCommand(gripper::stop, gripper).ignoringDisable(true));
    //operatorController.povDown().onTrue(new InstantCommand(chassis::stop, chassis).ignoringDisable(true));
    // operatorController.povLeft().onTrue(new InstantCommand(()-> {arm.stop(); arm.setState(ARM_ANGLE_STATES.IDLE);}, arm).ignoringDisable(true));

    // operatorController.rightBumper().onTrue(new InstantCommand(()-> arm.hadCalibrated()).ignoringDisable(true));
    
    // operatorController.rightSetting().onTrue(new InstantCommand(robot2Strip::setManualOrAuto).ignoringDisable(true));
    operatorController.leftSettings().onTrue(new InstantCommand(()-> chassis.setYaw(Rotation2d.kPi)).ignoringDisable(true));
    driverController.upButton().onTrue(new ElevatorCalibration(elevator));
    driverController.povLeft().onTrue(new InstantCommand(()->robotState = DemaciaRobotState.L4));
    driverController.povRight().onTrue(new InstantCommand(()->robotState = DemaciaRobotState.FEEDER));
    driverController.povDown().onTrue(new InstantCommand(()->robotState = DemaciaRobotState.L3));
    driverController.povUp().onTrue(new InstantCommand(()->robotState = DemaciaRobotState.STARTING));

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
    builder.addStringProperty("State", ()-> robotState.name(), null);
  }

  /**
   * This command is schedules at the start of teleop.
   * Look in {@link Robot} for more details.
   * @return the ommand that start at the start at enable
   */
  public Command getEnableInitCommand() {
    // return null;
    return new ElevatorCalibration(elevator);
    // return new ArmCalibration(arm);
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
      // arm.stop();
      // gripper.stop();
      // climb.stopClimb();
    }, chassis
    // }, chassis, arm, gripper, climb
    ).withName("initDisableCommand").ignoringDisable(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunCommand(()-> chassis.setSteerPositions(0), chassis);
  }
}

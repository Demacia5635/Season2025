// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.arm.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.robot2.arm.constants.ArmConstants;
import frc.robot.robot2.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot2.arm.constants.ArmConstants.ArmAngleMotorConstants;
import frc.robot.robot2.arm.constants.ArmConstants.GripperAngleMotorConstants;
import frc.robot.robot2.arm.constants.ArmConstants.MaxErrors;
import frc.robot.utils.Cancoder;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

import static frc.robot.robot2.arm.constants.ArmConstants.*;

/**
 * The Arm Subsystem
 * <br>
 * </br>
 * The subsytem is containing two motors and two sensors:
 * <br>
 * </br>
 * one motor is to change the angle of the motor and the other is to change the
 * angle of the gripepr
 * <br>
 * </br>
 * one sensor is digital and is being use to calibrate the angle and the other
 * is absolute sensor to the gripper angle in consta to the arm
 * <br>
 * </br>
 * the subsytem is worked on by state based system, the state variable is
 * deciding what the angles the arm needs to be
 * <br>
 * </br>
 * 
 * @see frc.robot.robot1.arm.constants.ArmConstants
 * @see frc.robot.robot1.arm.commands.ArmCommand
 */
public class Arm extends SubsystemBase {




  /** The motor that change the angle of the arm */
  private final TalonMotor armAngleMotor;
  /** The motor that change the angle of the gripper */
  private final TalonMotor gripperAngleMotor;

  /**canconder arm to elovetor */
  private final Cancoder armCancoder;
  /**
   * The absolute sensor that tells the gripper angle motor what is the real angle
   * of the gripper
   */
  private final DutyCycleEncoder gripperAngleAbsoluteSensor;



  /**
   * The state of the arm used in the
   * {@link frc.robot.robot1.arm.commands.ArmCommand} to tell the arm what angle
   * to go
   */
  public ARM_ANGLE_STATES state;

  private boolean hasArmAngleReachedTarget;
  private double lastArmAngleTarget;

  private boolean hasGripperAngleReachedTarget;

  /**
   * creates a new Arm, should only be one
   * <br>
   * </br>
   * configure the motors and the sensor and puts all the needed stuff at the
   * network tables
   */
  public Arm() {
    /* set the name of the subsystem */
    setName(NAME);

    /* configure the motors */
    armAngleMotor = new TalonMotor(ArmAngleMotorConstants.CONFIG);
    gripperAngleMotor = new TalonMotor(GripperAngleMotorConstants.CONFIG);

    /* configure the sensors */
    gripperAngleAbsoluteSensor = new DutyCycleEncoder(GripperAngleMotorConstants.ABSOLUTE_SENSOR_CHANNEL);


    /* configure the cancoder */
    armCancoder = new Cancoder(ArmAngleMotorConstants.CANCODER_CONFIG);





    /* make the default state to idle */
    state = ARM_ANGLE_STATES.IDLE;

    hasArmAngleReachedTarget = false;
    lastArmAngleTarget = Double.MAX_VALUE;

    hasGripperAngleReachedTarget = false;


    /* add to network tables everything that needed */
    addNT();
  }

 

  /**
   * add to network tables all the variables
   */
  public void addNT() {
    /* add to log the important stuff */
    LogManager.addEntry(getName() + "/Arm Abs Angle", this::getArmAngle, 4);
    LogManager.addEntry(getName() + "/Gripper Abs Angle", this::getGripperAngle, 4);
    
    LogManager.addEntry(getName() + "/Arm Angle", this::getArmAngleMotor, 4);
    LogManager.addEntry(getName() + "/Gripper Angle", this::getGripperAngleMotor, 4);
    LogManager.addEntry(getName() + "/IsReady", this::isReady, 4);

    /* add to smart dashboard the widgets of the talon motor */
    // SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME, armAngleMotor);
    // SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME, gripperAngleMotor);

    /* add to smart dashboard the coast and brake of both motors */
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set brake",
        new InstantCommand(() -> armAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + ArmAngleMotorConstants.NAME + "/arm angle set coast",
        new InstantCommand(() -> armAngleNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set brake",
        new InstantCommand(() -> gripperAngleNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData(getName() + "/" + GripperAngleMotorConstants.NAME + "/gripper angle set coast",
        new InstantCommand(() -> gripperAngleNeutralMode(false)).ignoringDisable(true));

    /* add state chooser through the netwrok tables */
    SendableChooser<ARM_ANGLE_STATES> stateChooser = new SendableChooser<>();
    stateChooser.addOption("L2", ARM_ANGLE_STATES.L2);
    stateChooser.addOption("L3", ARM_ANGLE_STATES.L3);
    stateChooser.addOption("L4", ARM_ANGLE_STATES.L4);
    stateChooser.addOption("Coral Station", ARM_ANGLE_STATES.CORAL_STATION);
    stateChooser.addOption("Starting", ARM_ANGLE_STATES.STARTING);
    stateChooser.addOption("Testing", ARM_ANGLE_STATES.TESTING);
    stateChooser.addOption("Idle", ARM_ANGLE_STATES.IDLE);
    stateChooser.onChange(state -> this.state = state);
    SmartDashboard.putData(getName() + "/Arm State Chooser", stateChooser);

    /*added pid motors to elastic */
    armAngleMotor.configPidFf(0);
    gripperAngleMotor.configPidFf(0);

    /* add the arm itself to the network tables */
    SmartDashboard.putData(this);
  }



  /**
   * set the state of the arm
   * 
   * @param state the wanted state
   */
  public void setState(ARM_ANGLE_STATES state) {
    this.state = state;
  }

  /**
   * get the current state of the arm
   * 
   * @return the current state
   */
  public ARM_ANGLE_STATES getState() {
    return state;
  }

  /**
   * set the arm angle motor neutral mode
   * 
   * @param isBrake is the motor needs to be brake (true -> brake; false -> coast)
   */
  public void armAngleNeutralMode(boolean isBrake) {
    armAngleMotor.setNeutralMode(isBrake);
  }

  /**
   * set the gripper angle motor neutral mode
   * 
   * @param isBrake is the motor needs to be brake (true -> brake; false -> coast)
   */
  public void gripperAngleNeutralMode(boolean isBrake) {
    gripperAngleMotor.setNeutralMode(isBrake);
  }

  /**
   * set power to the arm angle motor
   * 
   * @param power the wanted power from -1 to 1
   */
  public void armAngleMotorSetPower(double power) {
    armAngleMotor.setDuty(power);
  }

  /**
   * set the power to the gripper angle motor
   * 
   * @param power the wanted power from -1 to 1
   */
  public void gripperAngleMotorSetPower(double power) {
    gripperAngleMotor.setDuty(power);
  }

  /**
   * set power to both motors
   * 
   * @param armAnglePower     the wanted power for the arm angle motor from -1 to
   *                          1
   * @param gripperAnglePower the wanted power for the gripper angle motor from -1
   *                          to 1
   */
  public void setPower(double armAnglePower, double gripperAnglePower) {
    armAngleMotorSetPower(armAnglePower);
    gripperAngleMotorSetPower(gripperAnglePower);
  }

  /**
   * set the arm angle motor to position voltage with the target angle
   * 
   * @param targetAngle the wanted angle in radians
   * @see if the arm did not calibrate the request will not go through
   * @see if the target angle is below back limit the target will automaticly be
   *      the back limit
   * @see if the target angle is above the forward limit the target will be
   *      automaticly be the forward limit
   */
  public void armAngleMotorSetPositionVoltage(double targetAngle) {
    if (Double.isNaN(targetAngle)) {
      LogManager.log("arm target Angle is NaN", AlertType.kError);
      return;
    }

    if (lastArmAngleTarget != targetAngle) {
      hasArmAngleReachedTarget = false;
      lastArmAngleTarget = targetAngle;
    }

    if (targetAngle > getArmAngle()) {
      targetAngle += 2.5*MaxErrors.ARM_ANGLE_DOWN_ERROR;
    }

    if (Math.abs(targetAngle - getArmAngle()) <= Math.toRadians(1)) {
      hasArmAngleReachedTarget = true;
    }

    if (targetAngle < ArmAngleMotorConstants.BACK_LIMIT) {
      targetAngle = ArmAngleMotorConstants.BACK_LIMIT;
    }
    if (targetAngle > ArmAngleMotorConstants.FWD_LIMIT) {
      targetAngle = ArmAngleMotorConstants.FWD_LIMIT;
    }

    if (hasArmAngleReachedTarget) {
      if (getArmAngle() > targetAngle) {
        if (getArmAngle() - targetAngle > MaxErrors.ARM_ANGLE_UP_ERROR) {
          armAngleMotor.setPositionVoltage(targetAngle);
          hasArmAngleReachedTarget = false;
        } else {
          armAngleMotor.stopMotor();
        }
      } else {
        if (targetAngle - getArmAngle() > MaxErrors.ARM_ANGLE_DOWN_ERROR) {
          armAngleMotor.setPositionVoltage(targetAngle);
          hasArmAngleReachedTarget = false;
        } else {
          armAngleMotor.stopMotor();
        }
      }
    } else {
      armAngleMotor.setPositionVoltage(targetAngle);
    }

    // armAngleMotor.setPositionVoltage(targetAngle);
  }

  /**
   * set the gripper angle motor to position voltage with the target angle
   * 
   * @param targetAngle the wanted angle in radians
   * @see if the arm did not calibrate the request will not go through
   * @see if the target angle is below back limit the target will automaticly be
   *      the back limit
   * @see if the target angle is above the forward limit the target will be
   *      automaticly be the forward limit
   * @see if the arm angle is below a specific angle the gripepr will always want
   *      to go to the back limit
   */
  public void gripperAngleMotorSetPositionVoltage(double targetAngle) {
    targetAngle += GripperAngleMotorConstants.Angle_OFFSET;
    if (Double.isNaN(targetAngle)) {
      LogManager.log("gripper target Angle is NaN", AlertType.kError);
      return;
    }
    if (!gripperAngleAbsoluteSensor.isConnected()) {
      return;
    }

    // if (lastGripperAngleTarget != targetAngle) {
    //   hasGripperAngleReachedTarget = false;
    //   lastGripperAngleTarget = targetAngle;
    // }

    // if (Math.abs(targetAngle - getArmAngle()) <= Math.toRadians(1)) {
    //   hasGripperAngleReachedTarget = true;
    // }

    if (targetAngle < GripperAngleMotorConstants.BACK_LIMIT) {
      targetAngle = GripperAngleMotorConstants.BACK_LIMIT;
    }
    if (targetAngle > GripperAngleMotorConstants.FWD_LIMIT) {
      targetAngle = GripperAngleMotorConstants.FWD_LIMIT;
    }

    // if (hasGripperAngleReachedTarget) {
    //   if (getGripperAngle() > targetAngle) {
    //     if (getGripperAngle() - targetAngle > MaxErrors.GRIPPER_ANGLE_UP_ERROR) {
    //       gripperAngleMotor.setPositionVoltage(targetAngle);
    //       hasGripperAngleReachedTarget = false;
    //     } else {
    //       gripperAngleMotor.stopMotor();
    //     }
    //   } else {
    //     if (targetAngle - getGripperAngle() > MaxErrors.GRIPPER_ANGLE_DOWN_ERROR) {
    //       gripperAngleMotor.setPosition(targetAngle);
    //       hasGripperAngleReachedTarget = false;
    //     } else {
    //       gripperAngleMotor.stopMotor();
    //     }
    //   }
    // } else {
    //   gripperAngleMotor.setPositionVoltage(targetAngle);
    // }

    gripperAngleMotor.setPositionVoltage(getGripperAngleMotor() + targetAngle - getGripperAngle());
  }

  /**
   * set position voltage to both motors
   * 
   * @param armAngle     the wanted angle in radians for the arm angle motor
   * @param gripperAngle the wanted angle in radians for the gripper angle motor
   */
  public void setPositionVoltage(double armAngle, double gripperAngle) {
    armAngleMotorSetPositionVoltage(armAngle);
    gripperAngleMotorSetPositionVoltage(gripperAngle);
  }

  /**
   * set the angle motor position
   * 
   * @param angle the new value the arm angle motor position will be
   */
  public void armAngleSetPosition(double angle) {
    armAngleMotor.setPosition(angle);
  }

  /**
   * stop both motorss
   */
  public void stop() {
    armAngleMotor.stopMotor();
    gripperAngleMotor.stopMotor();
  }

  /**
   * get is the motors closed loop error are less than the max errors
   * 
   * @return is the motors at the right angles
   */
  public boolean isReady() {
    return hasArmAngleReachedTarget && hasGripperAngleReachedTarget;
  }

  /**
   * get the arm angle
   * 
   * @return the arm angle motor position, position in radians
   */
  public double getArmAngleMotor() {
    return armAngleMotor.getCurrentPosition();
  }

   /** armCancoder get angle */
   public double getArmAngle(){
    return armCancoder.getCurrentAbsPosition()-ArmAngleMotorConstants.Angle_OFFSET;
  }

  /**
   * @return the gripper angle motor position, position in radians
   */
  public double getGripperAngleMotor() {
    return gripperAngleMotor.getCurrentPosition();
  }

  /**
   * get the gripper angle absolute sensor
   * 
   * @return the gripper angle in radians
   */
  public double getGripperAngle() {
    return (gripperAngleAbsoluteSensor.get() * 2 * Math.PI) - GripperAngleMotorConstants.Angle_OFFSET;
  }



  /**
   * the init sendable of the command
   * this is for all the stuff that can not be in the log
   * 
   * @param builder the builder of the subsystem
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addStringProperty("state", () -> getState().name(), null);
  }

  /**
   * This function runs every cycle
   * <br>
   * </br>
   * the function check if the absolute gripper angle sensor is connected
   * <br>
   * </br>
   * the function set the gripper angle motor position to the absolute sensor
   */
  @Override
  public void periodic() {
    /* check if the gripper angle absolute sensor is connected */
    if (!gripperAngleAbsoluteSensor.isConnected()) {
      LogManager.log("Gripper Angle Encoder is not connected", AlertType.kError);
    }

    /* set the gripper angle motor position to the gripper angle absolute sensor */
    // gripperAngleMotor.setPosition(getGripperAngle());
  }
}

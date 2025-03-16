// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot2.arm.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.MathUtil;
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
 * @see frc.robot.robot2.arm.constants.ArmConstants
 * @see frc.robot.robot2.arm.commands.ArmCommand
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


  private boolean hasArmAngleReachedTarget;
  private double lastArmAngleTarget;
  private double lastGripperAngleTarget;

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
    
    // LogManager.addEntry(getName() + "/Arm Angle", this::getArmAngleMotor, 4);
    // LogManager.addEntry(getName() + "/Gripper Angle", this::getGripperAngleMotor, 4);
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

    /* add the arm itself to the network tables */
    // SmartDashboard.putData(this);
  }

  // public int getHowMuchReady(int divisions) {
  //   double max = state.armAngle;
  //   double min = ARM_ANGLE_STATES.STARTING.armAngle;
  //   double range = max - min;
  //   double part = range / divisions;
  //   double value = getState().armAngle;

  //   if (range <= 0) {
  //     return 0;
  //   }

  //   if (value <= max) {
  //     return 1 / divisions;
  //   }
    
  //   for (int i = 0; i < divisions; i++) {
  //     if (value >= i*part + min && value < (i+1)*part + min) {
  //       return (divisions - i) / divisions;
  //     }
  //   }

  //   return 0;
  // }

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
 
    // if (lastArmAngleTarget != targetAngle) {
    //   hasArmAngleReachedTarget = false;
    //   lastArmAngleTarget = targetAngle;
    // }

    // if (targetAngle > getArmAngle()) {
    //   targetAngle += 2.5*MaxErrors.ARM_ANGLE_DOWN_ERROR;
    // }

    if (Math.abs(targetAngle - getArmAngle()) <= Math.toRadians(1)) {
      hasArmAngleReachedTarget = true;
    }

    if (targetAngle != lastArmAngleTarget) {
      lastArmAngleTarget = targetAngle;
      targetAngle += getArmAngleMotor() - getArmAngle();
    } else {
      targetAngle = lastArmAngleTarget + getArmAngleMotor() - getArmAngle();
    }
   
    armAngleMotor.setMotionMagic(targetAngle);
    

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
    if (Double.isNaN(targetAngle)) {
      LogManager.log("gripper target Angle is NaN", AlertType.kError);
      return;
    }
    
    /* check if the gripper angle absolute sensor is connected */
    if (!gripperAngleAbsoluteSensor.isConnected()) {
      LogManager.log("Gripper Angle Encoder is not connected", AlertType.kError);
      return;
    }

    targetAngle -= armAngleMotor.getCurrentVelocity() * 0.5;

    // if (lastGripperAngleTarget != targetAngle) {
    //   hasGripperAngleReachedTarget = false;
    //   lastGripperAngleTarget = targetAngle;
    // }

    // if (Math.abs(targetAngle - getArmAngle()) <= Math.toRadians(1)) {
    //   hasGripperAngleReachedTarget = true;
    // }

    if (targetAngle < GripperAngleMotorConstants.BACK_LIMIT) {
      targetAngle = GripperAngleMotorConstants.BACK_LIMIT + 0.1;
    }
    if (targetAngle > GripperAngleMotorConstants.FWD_LIMIT) {
      targetAngle = GripperAngleMotorConstants.FWD_LIMIT - 0.1;
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
    if (targetAngle != lastGripperAngleTarget) {
      lastGripperAngleTarget = targetAngle;
      targetAngle += getGripperAngleMotor() - getGripperAngle();
    } else {
      targetAngle = lastGripperAngleTarget + getGripperAngleMotor() - getGripperAngle();
    }
    gripperAngleMotor.setPositionVoltage(targetAngle);
  }

  /**
   * set position voltage to both motors
   * 
   * @param armAngle     the wanted angle in radians for the arm angle motor
   * @param gripperAngle the wanted angle in radians for the gripper angle motor
   */
  public void setPositionVoltage(double armAngle, double gripperAngle) {
    gripperAngleMotorSetPositionVoltage(gripperAngle);
    armAngleMotorSetPositionVoltage(armAngle);
    
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
   * @return griper motor velocity
   */
  public double getGripperVel(){
    return gripperAngleMotor.getCurrentVelocity();
  }

  /**
   * @return arm motor velocity
   */
  
  public double getArmVel(){
    return armAngleMotor.getCurrentVelocity();
  }

  

  /**
   * get is the motors closed loop error are less than the max errors
   * 
   * @return is the motors at the right angles
   */
  public boolean isReady() {
    return hasArmAngleReachedTarget && Math.abs(gripperAngleMotor.getCurrentClosedLoopError()) < 0.03;
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
    return MathUtil.angleModulus(armCancoder.getCurrentAbsPosition()-ArmAngleMotorConstants.Angle_OFFSET);
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
    return MathUtil.angleModulus((gripperAngleAbsoluteSensor.get() * 2 * Math.PI) - GripperAngleMotorConstants.Angle_OFFSET);
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

    if (Math.abs(getArmAngle()) < Math.PI/2) {
      armAngleMotor.changeSlot(0);
    } else {
      armAngleMotor.changeSlot(1);
    }

    /* set the gripper angle motor position to the gripper angle absolute sensor */
    // gripperAngleMotor.setPosition(getGripperAngle());
  }
}

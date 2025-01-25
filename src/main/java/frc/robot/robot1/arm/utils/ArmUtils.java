// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.utils;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The arm utils
 * here all the calculation of the arm
 */
public class ArmUtils {

    /**
     * function to calculate the angles of the arms by distance and height
     * @param distance the distance from the target to the robot in meters
     * @param height the height of the target to the floor in meters
     * @exception math if the calculation is bigger than 1 or less than -1 return base angle for the arm angle and back limit for the gripper angle
     * @return the needed angles of the arm the first angle is the arm angle and the second is the gripper angle both in radians
     */
    public static Pair<Double, Double> calculateAngles(double distance, double height) {

        /* set the hypotenuse of the triangle */
        double relativeHeight = CalculationsConstants.BASE_HEIGHT - height;
        Translation2d hypotenuse = new Translation2d(distance, relativeHeight);

        /* check if the calculation is not acos of more and than 1 and not less than -1 */
        if (Math.abs(
            (Math.pow(CalculationsConstants.ARM_2_LEN, 2) + Math.pow(CalculationsConstants.ARM_1_LEN, 2) - Math.pow(hypotenuse.getNorm(), 2)) /
            (2 * CalculationsConstants.ARM_1_LEN * CalculationsConstants.ARM_2_LEN)
        ) >= 1) {
            return new Pair<Double,Double>(ArmAngleMotorConstants.BASE_ANGLE, GripperAngleMotorConstants.BACK_LIMIT);
        }

        /* using cosines law calculate both angles and modify them to match the needed angles for the motors  */
        return new Pair<Double,Double>(
            Math.acos(
                (Math.pow(CalculationsConstants.ARM_1_LEN, 2) + Math.pow(hypotenuse.getNorm(), 2) - Math.pow(CalculationsConstants.ARM_2_LEN, 2)) / 
                (2 * CalculationsConstants.ARM_1_LEN * hypotenuse.getNorm())) +
                0.5 * Math.PI - hypotenuse.getAngle().getRadians(),
            2 * Math.PI - 
            Math.acos(
                (Math.pow(CalculationsConstants.ARM_2_LEN, 2) + Math.pow(CalculationsConstants.ARM_1_LEN, 2) - Math.pow(hypotenuse.getNorm(), 2)) / 
                (2 * CalculationsConstants.ARM_2_LEN * CalculationsConstants.ARM_1_LEN))
        );
    }
}

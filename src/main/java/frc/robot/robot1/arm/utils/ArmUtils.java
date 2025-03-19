// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.utils;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

import edu.wpi.first.math.Pair;

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

        double relativeHeight = CalculationsConstants.BASE_HEIGHT - height;

        return new Pair<Double,Double>(
            (
                Math.PI 
                / 2
            ) 
            - Math.atan(relativeHeight/distance) 
            + Math.acos(
                (
                    Math.pow(CalculationsConstants.ARM_1_LEN, 2) 
                    + Math.pow(distance, 2) + Math.pow(relativeHeight, 2) 
                    - Math.pow(CalculationsConstants.ARM_2_LEN, 2)
                ) / (
                    2 
                    * CalculationsConstants.ARM_1_LEN 
                    * Math.sqrt(
                        Math.pow(distance, 2) 
                        + Math.pow(relativeHeight, 2)
                    )
                )
            ), 
            2 * Math.PI
            - (
                Math.acos(
                    (
                        Math.pow(CalculationsConstants.ARM_1_LEN, 2)
                        + Math.pow(CalculationsConstants.ARM_2_LEN, 2)
                        - Math.pow(distance, 2)
                        - Math.pow(relativeHeight, 2)
                    ) / (
                        2
                        * CalculationsConstants.ARM_1_LEN
                        * CalculationsConstants.ARM_2_LEN
                    )
                )
            )
        );
    }
}

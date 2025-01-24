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
     * this function calculate the needed angles of the arm
     * @param dist distance from the wanted target in meters
     * @param height height of the target in meters
     * @return pair of double that are the needed angles in radians
     */
    public static Pair<Double, Double> calcAngles(double dist, double height) {
        height = height - CalculationsConstants.BASE_HEIGHT;
        double angleToPoint = Math.toDegrees(Math.atan(height / dist));
        double hypotenuse = height / Math.sin(Math.toRadians(angleToPoint));
        if (hypotenuse > (CalculationsConstants.ARM_1_LEN + CalculationsConstants.ARM_2_LEN)
                || hypotenuse < (CalculationsConstants.ARM_1_LEN - CalculationsConstants.ARM_2_LEN)) {
            return null;
        }
        double jointA = 90 - angleToPoint
                - Math.toDegrees(Math.acos(
                        ((CalculationsConstants.ARM_1_LEN * CalculationsConstants.ARM_1_LEN) + (hypotenuse * hypotenuse)
                                - (CalculationsConstants.ARM_2_LEN * CalculationsConstants.ARM_2_LEN))
                                / (2 * CalculationsConstants.ARM_1_LEN * hypotenuse)));
        double jointB = 180
                - Math.toDegrees(Math.acos(((CalculationsConstants.ARM_1_LEN * CalculationsConstants.ARM_1_LEN)
                        + (CalculationsConstants.ARM_2_LEN * CalculationsConstants.ARM_2_LEN)
                        - (hypotenuse * hypotenuse))
                        / (2 * CalculationsConstants.ARM_1_LEN * CalculationsConstants.ARM_2_LEN)));
        return new Pair<Double, Double>(jointA, jointB);
    }

    // public double[] calcAngles(double dist, double hight){
    // hight = hight-HEIGHT;
    // angleToPoint = Math.toDegrees(Math.atan(hight/dist));
    // arm3 = hight/Math.sin(Math.toRadians(angleToPoint));
    // if (arm3 > (ARM1 + ARM2) || arm3 < (ARM1 - ARM2)){
    // return null;
    // }
    // jointA = 90 - angleToPoint - Math.toDegrees(Math.acos(((ARM1*ARM1) +
    // (arm3*arm3) - (ARM2*ARM2))/(2*ARM1*arm3)));
    // jointB = 180 - Math.toDegrees(Math.acos(((ARM1*ARM1) + (ARM2*ARM2) -
    // (arm3*arm3))/(2*ARM1*ARM2)));
    // return new double[]{jointA,jointB};
    // }
}

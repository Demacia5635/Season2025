// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot1.arm.utils;

import static frc.robot.robot1.arm.constants.ArmConstants.*;

/** Add your docs here. */
public class ArmUtils {

    public static ArmState calcAngles(double dist, double hight) {
        hight = hight - TABLE_HIGHT;
        double angleToPoint = Math.toDegrees(Math.atan(hight / dist));
        double arm3 = hight / Math.sin(Math.toRadians(angleToPoint));
        if (arm3 > (ARM1 + ARM2) || arm3 < (ARM1 - ARM2)) {
            return null;
        }
        double jointA = 90 - angleToPoint
                - Math.toDegrees(Math.acos(((ARM1 * ARM1) + (arm3 * arm3) - (ARM2 * ARM2)) / (2 * ARM1 * arm3)));
        double jointB = 180
                - Math.toDegrees(Math.acos(((ARM1 * ARM1) + (ARM2 * ARM2) - (arm3 * arm3)) / (2 * ARM1 * ARM2)));
        return new ArmState(jointA, jointB);
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

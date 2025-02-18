// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils;

/** Add your docs here. */
public class Trapezoid {
    private double maxVelocity;
    private double accel;
    private double maxDelta;
    private double finishVel;

    public Trapezoid(double maxVelocity, double accel, double finishVel){
        this.maxVelocity = maxVelocity;
        this.accel = accel;
        this.finishVel = finishVel;
        this.maxDelta = accel * 0.02;
    }

    private double getTime(double currentVelocity){
        return (currentVelocity - finishVel) / accel;
    }
    public double calculate(double currentVelocity, double distanceLeft){
        if(canAccel(currentVelocity, distanceLeft)) return currentVelocity + maxDelta;
        if(canKeep(currentVelocity, distanceLeft)) return currentVelocity;
        return currentVelocity - maxDelta;

    }

    private boolean canKeep(double curVelocity, double distanceLeft){
        return curVelocity * getTime(curVelocity) < distanceLeft;
    }
    private boolean canAccel(double curVelocity, double distanceLeft){
        double t = getTime(curVelocity + maxDelta);
        
        return curVelocity*t + ( 0.5 * accel * Math.pow(t, 2)) < distanceLeft && curVelocity + maxDelta < maxVelocity;   
    }
    private double accelDistance(double curVelocity, double distanceLeft){
        double t = getTime(curVelocity + maxDelta);
        if(t < 0) {
            return -curVelocity*t + (0.5 * accel * t * t);
        } else {
            return curVelocity*t - (0.5 * accel * t * t);
        }
    }
}

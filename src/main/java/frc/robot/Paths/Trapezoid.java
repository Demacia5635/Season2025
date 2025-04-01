// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Paths;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class Trapezoid {

    private final double maxVelocity;
    private final double maxAcceleration;
    private final double maxJerk;
    private final double finalVelocity;

    public Trapezoid(double maxVelocity, double maxAcceleration, double maxJerk, double finalVelocity){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        this.finalVelocity = finalVelocity;
    }

    
    public Trapezoid(double maxVelocity, double maxAcceleration, double finalVelocity){
        this(maxVelocity, maxAcceleration, Double.POSITIVE_INFINITY, finalVelocity);
    }

    public Trapezoid(double maxVelocity, double maxAcceleration){
        this(maxVelocity, maxAcceleration, Double.POSITIVE_INFINITY, 0);
    }
    
    public double calculate(double currentVelocity, double distanceLeft){
        double absDistanceLeft = Math.abs(distanceLeft);
        double absCurrentVelocity = Math.abs(currentVelocity);
        double absFinalVelocity = Math.abs(finalVelocity);
        double stoppingDistance = getStoppingDistance(absCurrentVelocity, absFinalVelocity);

        double v = 0;
        double a = 0;

        if(absDistanceLeft < stoppingDistance){
            a = -maxAcceleration;
        }
        else{
            if(absCurrentVelocity < maxVelocity) a = maxAcceleration;
            else a = 0;
        }

        v = currentVelocity + (a * 0.02);
        v = MathUtil.clamp(v, -maxVelocity, maxVelocity) * Math.signum(distanceLeft);
        a *= Math.signum(distanceLeft);

        return v;
        

    };

    private double getStoppingDistance(double velocity, double finalVelocity){
        if(velocity < finalVelocity) return 0; //no need to deAccelerate to stop

        double accelDistanceToStop = ((velocity * velocity) - (finalVelocity * finalVelocity)) / (2 * maxAcceleration);

        if(maxJerk == Double.POSITIVE_INFINITY) return accelDistanceToStop;

        double jerkDistanceToStop = (maxAcceleration * maxAcceleration) / (2 * maxJerk);

        return accelDistanceToStop + jerkDistanceToStop;
        
        
    }



}

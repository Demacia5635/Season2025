// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */



public class Leg extends Segment{
    Translation2d totalVector;
    final Translation2d velDirection;
    /**
     * creates a leg type segment
     * @param p1 the first point of the leg
     * @param p2 the last point of the leg
     */
    public Leg(Translation2d p1, Translation2d p2)
    {
        super(p1, p2);
        totalVector = p2.minus(p1);
        velDirection = totalVector.div(totalVector.getNorm());
    }

    @Override
    public Translation2d calcVector(Translation2d position, double velocity)
    {
       // if (p2.getDistance(position) <= 0.2) return new Translation2d(velocity, position.minus(p2).getAngle());
        Translation2d relativePos = position.minus(p2);
        double diffAngleMaxed = Math.min(15, p1.minus(p2).getAngle().minus(relativePos.getAngle()).getDegrees());

        return new Translation2d(velocity, relativePos.times(-1).getAngle().minus(Rotation2d.fromDegrees(diffAngleMaxed)));

    }

    @Override
    public double distancePassed(Translation2d position)
    {
        Translation2d relativePos = position.minus(p1);

        //double distanceMoved = (relativePos.getX() * velDirection.getX()) + (relativePos.getY()*velDirection.getY());
        return Math.abs(relativePos.getNorm());
    }

    @Override
    public double getLength()
    {
        return totalVector.getNorm();
    }

   

    @Override
    public String toString() {
        return "\n~Leg~\np1 : " + p1 + "\np2 : " + p2 + "\nTotalVector : " + totalVector;
    }
}

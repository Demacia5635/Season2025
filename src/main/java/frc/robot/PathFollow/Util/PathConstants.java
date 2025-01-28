// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import static frc.robot.PathFollow.Util.PathsConstants.STATION_RADIUS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
class PathsConstants {
    private static PathsConstants pc = new PathsConstants();
    public final static double FIELD_LENGTH = 16.54; // in meters
    public final static double FIELD_HEIGH = 8.21; // in meters
    public final static double DISTANCE_OFFSET = 0.01;;
    public final static double MAX_VELOCITY = 4.1;
    public final static double ACCEL = 8;
    public final static double MAX_ROTATION_VELOCITY = 180;
    public final static double ROTATION_ACCEL = 360;
    public final static double FINISH_OFFSET = 0.05;
    public final static double MIN_SEGMENT_LENGTH = 0.15;
    public final static double MAX_RADIAL_ACCEL = -1;

    public final static Translation2d STATION_CENTER = new Translation2d((1.57 + 1.8)/Math.sqrt(2),(1.57 + 1.8)/Math.sqrt(2));

    private PathsConstants()
    {
        PathsConstants.pc = this;
        STATIONS = new Pose2d[12];
        for(int i = 0; i < 12; i++)
        {
            STATIONS[i] = new Pose2d(new Translation2d(Math.cos((-(Math.PI*2)/12)* i - Math.PI/2) ,Math.sin((-(Math.PI*2)/12)* i - Math.PI/2)).times(STATION_RADIUS).plus(new Translation2d(4,4)),new Rotation2d(0));
        }
    }

    
    public final static double STATION_RADIUS = 1.2 + 0.6;
    public static Pose2d[] STATIONS; //please arrange it clockwise

    public static double dot_prod(Translation2d v1,Translation2d v2)
    {
        return v1.getX()*v2.getX() + v1.getY()*v2.getY();
    }

    public static double cross_prod(Translation2d v1,Translation2d v2)
    {
        return v1.getX()*v2.getY() - v1.getY()*v2.getX();
    }

    public static double proj_scalar_pow2(Translation2d v1, Translation2d v2){
        double dot1 = dot_prod(v1, v2);

        double v1_len_squared = v1.getX()*v1.getX() + v1.getY()*v1.getY();
        
        double x = v1.getX()*(dot1/v1_len_squared);
        double y = v1.getY()*(dot1/v1_len_squared);
        System.out.println("Vector proj : " + new Translation2d(x,y));
        return x*x + y*y;
    }
    
    //Count segment and circle intersections

    public static int cSegCircleInter(Translation2d a, Translation2d b, Translation2d c_pos, double r)
    {
        //unable to calculate m
        if(a.getX() == b.getX() || a.getY() == b.getY())
        {
            if(Math.abs(a.getY() - c_pos.getY()) == r)
                return 1;
            if(Math.abs(a.getX() - c_pos.getX()) == r)
                return 1;
            if(Math.abs(a.getY() - c_pos.getY()) < r)
                return 2;
            if(Math.abs(a.getX() - c_pos.getX()) < r)
                return 2;
        }


        Translation2d ab = a.minus(b);
        Translation2d a_cpos = c_pos.minus(a);

        double r_squared = r*r;
        double hypo_squared = a_cpos.getX()*a_cpos.getX() + a_cpos.getY()*a_cpos.getY();

        double proj_squared = proj_scalar_pow2(ab, a_cpos);
        
        double min_x = Math.min(a.getX(), b.getX());
        double max_x = Math.max(a.getX(), b.getX());


        if(hypo_squared - proj_squared == r_squared)
        {
            if(min_x < (c_pos.getX() - r) && max_x > (c_pos.getX() + r)) 
            {
                return 1;
            }
            else{
                //check x of line intersection, make sure x is in range of a&b's x
                // y = mx + b
                double ab_m = (a.getY() - b.getY()) / (a.getX() - b.getX());
                double ab_m_norm = -1 / ab_m;

                double ab_b = -ab_m*a.getX() + a.getY();
                double ab_b_norm = -ab_m_norm*c_pos.getX() + c_pos.getY();

                double inter_x = (ab_b_norm - ab_b) / (ab_m - ab_m_norm);
                
                System.out.println("Inter x : " + inter_x);

                if(inter_x > min_x && inter_x < max_x)
                {
                    return 1;
                }
                else{
                    return 0;
                }
                
            }
            
        }
        else{
            if(hypo_squared - proj_squared < r_squared)
            {
                if(min_x < (c_pos.getX() - r) && max_x > (c_pos.getX() + r)) 
                {
                    return 2;
                }
                else
                {
                    Translation2d b_cpos = b.minus(c_pos);
                    double hypo_b_squared = b_cpos.getX()*b_cpos.getX() + b_cpos.getY()*b_cpos.getY();
                    if (hypo_b_squared < r_squared && hypo_squared < r_squared)
                        return -1; //Line inside circle

                    //points on circle
                    if (hypo_b_squared == r_squared && hypo_squared == r_squared)
                        return 2;
                    
                    // point 1 inside circle, point b outside circle
                    if(hypo_b_squared < r_squared && hypo_squared >= r_squared || hypo_b_squared >= r_squared && hypo_squared < r_squared)
                        return 1;
                    
                    //check x of line intersection, make sure x is in range of a&b's x
                    // y = mx + b


                    double ab_m = (a.getY() - b.getY()) / (a.getX() - b.getX());
                    double ab_m_norm = -1 / ab_m;

                    double ab_b = -ab_m*a.getX() + a.getY();
                    double ab_b_norm = -ab_m_norm*c_pos.getX() + c_pos.getY();

                    double inter_x = (ab_b_norm - ab_b) / (ab_m - ab_m_norm);

                    if(inter_x > min_x && inter_x < max_x)
                    {
                        return 2;
                    }
                    else{
                        return 0;
                    }            
                }

            }
            return 0;
        }

    }

}
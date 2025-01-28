// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PathFollow.PathFollow;

import static frc.robot.PathFollow.Util.PathsConstants.STATIONS;
import static frc.robot.PathFollow.Util.PathsConstants.STATION_CENTER;
import static frc.robot.PathFollow.Util.PathsConstants.STATION_RADIUS;

import java.nio.file.Paths;
import java.util.ArrayList;


/** Add your docs here. */
public class StationNav {
    
    private static Translation2d getEntryPoint(Translation2d curPose){
        double minDistance = Integer.MAX_VALUE;
        int minIndex = -1;
        for (int i = 0; i < PathsConstants.STATIONS.length; i++) {
            Pose2d curStation = PathsConstants.STATIONS[i];
            if(curPose.getDistance(curStation.getTranslation()) < minDistance){
                minDistance = curPose.getDistance(curStation.getTranslation());
                minIndex = i;
            }
        }
        return PathsConstants.STATIONS[minIndex].getTranslation();
    }
    private static int getIndex(Translation2d pos){
        
        for(int i = 0; i < PathsConstants.STATIONS.length; i++){
            if(pos == PathsConstants.STATIONS[i].getTranslation()) return i;
        }
        return -1;
    }

    private static void reverseArray(Translation2d[] array){
        for (int i = 0; i < array.length / 2; i++) {
            Translation2d t = array[i];
            array[i] = array[array.length - 1 - i];
            array[array.length - 1 - i] = t;
        }


    }
    private static Translation2d[] calcPositionPoints(Pose2d curPose, Translation2d finalPoint){
        Translation2d entryPoint = getEntryPoint(curPose.getTranslation());
        int entryPointIndex = getIndex(entryPoint);
        int finalPointIndex = getIndex(finalPoint);
        
        ArrayList<Translation2d> points = new ArrayList<>();
        Translation2d[] pointsArray;

        if(entryPointIndex < finalPointIndex){
            for(int i = entryPointIndex; i < finalPointIndex; i++){
                points.add(PathsConstants.STATIONS[i].getTranslation());
            }
            pointsArray = (Translation2d[]) points.toArray();
        }
        else{
            for(int i = finalPointIndex; i < entryPointIndex; i++){
                points.add(PathsConstants.STATIONS[i].getTranslation());
            }
            pointsArray = (Translation2d[]) points.toArray();
            reverseArray(pointsArray);
        }

        return pointsArray;
    }

    // public static PathFollow goToScore(Pose2d curPose, Pose2d scorePose){
    //     Rotation2d angleToScore = scorePose.getRotation();
    //     Translation2d[] points = calcPositionPoints(curPose, scorePose.getTranslation());
    //     PathPoint[] PathPoints = new PathPoint[points.length];
    //     for(int i = 0; i < PathPoints.length; i++){
    //         PathPoints[i] = new PathPoint(points[i], angleToScore);
    //     }
    //     return new PathFollow(PathPoints);
    // }

    



    private static PathPoint[] bridgeClock(int closeInit,int closeFin, Translation2d init, Pose2d fin)
    {
        int diff = Math.abs(closeFin-closeInit);
         int cPoints = closeFin > closeInit ? diff + 3 : STATIONS.length - diff + 3; 
        PathPoint[] points = new PathPoint[cPoints];
        points[0] = new PathPoint(init,fin.getRotation());

        for(int i = 1,j=closeInit; i < cPoints - 2; i++,j++){
            // System.out.println("index : " + j%(STATIONS.length));
            points[i] = new PathPoint(STATIONS[j%(STATIONS.length)].getTranslation(), fin.getRotation());
        }
        
        points[cPoints - 2] = new PathPoint(STATIONS[closeFin].getTranslation(), fin.getRotation());
        points[cPoints - 1] = new PathPoint(fin.getTranslation(), fin.getRotation());

        return points;

    }


    private static PathPoint[] bridgeCounter(int closeInit,int closeFin, Translation2d init, Pose2d fin)
    {
        int diff = Math.abs(closeFin-closeInit);
        int cPoints = closeInit > closeFin ? diff + 3: STATIONS.length - diff + 3;// stations - diff + initclose + fin+ init
        PathPoint[] points = new PathPoint[cPoints];
        points[0] = new PathPoint(init,fin.getRotation());

        for(int i = 1,j=closeInit; i < cPoints - 2; i++,j--)
        {
            // System.out.println("index : " + j%(STATIONS.length));
            points[i] = new PathPoint(STATIONS[j].getTranslation(), fin.getRotation());
            if(j == 0)
                j = STATIONS.length;
        }

        points[cPoints - 2] = new PathPoint(STATIONS[closeFin].getTranslation(), fin.getRotation());
        points[cPoints - 1] = new PathPoint(fin.getTranslation(), fin.getRotation());
        return points;
    }

    private static int shiftClock(int tan,int close,Translation2d dest)
    {
        int station = tan;
        
        //intersections counter
        int inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        while(station != close && (inter == 2 || inter == -1)){
            //move clockwise
            station = (station + 1) % STATIONS.length;
            inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        } 

        return station;
    }

    private static int shiftCounter(int tan,int close,Translation2d dest)
    {
        int station = tan;
        
        //intersections counter
        int inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        while(station != close && (inter == 2 || inter == -1)){
            //move counter-clockwise
            if(station - 1 >= 0)
                station--;
            else
                station = STATIONS.length - 1;
            inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        } 

        return station;
    }
    
    private static Boolean decideCounter(int station1,int station2)
    {
        int diff = Math.abs(station1 - station2);
        if(station1 < station2)
        {
            return diff > STATIONS.length/2;
        }
        else
        {
            return !(diff > STATIONS.length/2);
        }
    }

    private static int shiftTan(int tan,int close,Translation2d dest)
    {
        if(decideCounter(tan, close))
            return shiftCounter(tan, close, dest);
        else
            return shiftClock(tan, close, dest);
    }

    //will return a tuple
    private static int[] optimzeGates(int tanInit,int tanFin,int closeInit,int closeFin)
    {
        int diff_closeclose = Math.min(STATIONS.length - Math.abs(closeFin-closeInit),Math.abs(closeFin-closeInit));
        int diff_tanclose = Math.min(STATIONS.length - Math.abs(tanInit-closeFin),Math.abs(tanInit-closeFin));
        int diff_closetan = Math.min(STATIONS.length - Math.abs(closeInit-tanFin),Math.abs(closeInit-tanFin));
        int diff_tantan = Math.min(STATIONS.length - Math.abs(tanInit-tanFin),Math.abs(tanInit-tanFin));

        int lowest_diff = Math.min(Math.min(diff_closeclose, diff_tanclose),Math.min(diff_closetan, diff_tantan));

        int[] gates = new int[2];
        if(lowest_diff == diff_tantan)
        {
            System.out.println("tantan");
            gates[0] = tanInit;
            gates[1] = tanFin;
            return gates;
        }

        if(lowest_diff == diff_tanclose)
        {  
            System.out.println("tanclose");
            gates[0] = tanInit;
            gates[1] = closeFin;
            return gates;
        }
        if(lowest_diff == diff_closetan)
        {
            System.out.println("closetan");
            gates[0] = closeInit;
            gates[1] = tanFin;
            return gates;
        }

        if(lowest_diff == diff_closeclose){
            System.out.println("closeclose");
            gates[0] = closeInit;
            gates[1] = closeFin;
            return gates;
        }
        
        return gates;

    }

    public static PathPoint[] genLineByDis(Translation2d initial,Pose2d fin,double velocity){
        
        // if(PathsConstants.cSegCircleInter(initial, fin.getTranslation(), initial, velocity))

        int closeInit = 0;
        int closeFin = 0;

        int tanInit = 0;
        int tanFin = 0;

        //secound tangents
        int tanInit2 = 0;
        int tanFin2 = 0;
        
        for(int i = 1; i < STATIONS.length; i++)
        {
            Pose2d station = STATIONS[i];//TODO:optimize this loop

            double d_init = station.getTranslation().minus(initial).getNorm();
            double d_initclose = initial.minus(
                STATIONS[closeInit].getTranslation()).getNorm();

            double d_fin = station.getTranslation().minus(fin.getTranslation()).getNorm();
            double d_finclose = fin.getTranslation().minus(
                STATIONS[closeFin].getTranslation()).getNorm();

            double dot_init = PathsConstants.dot_prod(initial.minus(station.getTranslation()), station.getTranslation().minus(PathsConstants.STATION_CENTER));
            double dot_initclose = PathsConstants.dot_prod(initial.minus(STATIONS[tanInit].getTranslation()), STATIONS[tanInit].getTranslation().minus(PathsConstants.STATION_CENTER));

            double dot_initclose2 = PathsConstants.dot_prod(initial.minus(STATIONS[tanInit2].getTranslation()), STATIONS[tanInit2].getTranslation().minus(PathsConstants.STATION_CENTER));

            double dot_fin = PathsConstants.dot_prod(fin.getTranslation().minus(station.getTranslation()), station.getTranslation().minus(PathsConstants.STATION_CENTER));
            double dot_finclose = PathsConstants.dot_prod(fin.getTranslation().minus(STATIONS[tanFin].getTranslation()), STATIONS[tanFin].getTranslation().minus(PathsConstants.STATION_CENTER));

            double dot_finclose2 = PathsConstants.dot_prod(fin.getTranslation().minus(STATIONS[tanFin2].getTranslation()), STATIONS[tanFin2].getTranslation().minus(PathsConstants.STATION_CENTER));
            
            if(Math.abs(dot_init) < Math.abs(dot_initclose))
                tanInit = i;
            else
            {
                if(Math.abs(dot_init) <= Math.abs(dot_initclose2))
                    tanInit2 = i;
            }

            if(Math.abs(dot_fin) < Math.abs(dot_finclose))
                tanFin = i;
            else
            {
                if(Math.abs(dot_fin) <= Math.abs(dot_finclose2))
                    tanFin2 = i;

            }

            if (d_init < d_initclose)
                closeInit = i;
            if(d_fin < d_finclose)
                closeFin = i;
        }

        Translation2d tanToInit = initial.minus(STATIONS[tanInit].getTranslation());
        Translation2d tanToFin = fin.getTranslation().minus(STATIONS[tanInit].getTranslation());

        Translation2d tanToInit2 = initial.minus(STATIONS[tanInit2].getTranslation());
        Translation2d tanToFin2 = fin.getTranslation().minus(STATIONS[tanInit2].getTranslation());

        if(PathsConstants.dot_prod(tanToInit2, tanToFin2) < PathsConstants.dot_prod(tanToInit, tanToFin))
            tanInit = tanInit2;

        tanToInit = initial.minus(STATIONS[tanFin].getTranslation());
        tanToFin = fin.getTranslation().minus(STATIONS[tanFin].getTranslation());

        tanToInit2 = initial.minus(STATIONS[tanFin2].getTranslation());
        tanToFin2 = fin.getTranslation().minus(STATIONS[tanFin2].getTranslation());

        if(PathsConstants.dot_prod(tanToInit2, tanToFin2) < PathsConstants.dot_prod(tanToInit, tanToFin))
            tanFin = tanFin2;

        System.out.println("tanInit : " + STATIONS[tanInit]);
        System.out.println("tanFin : " + STATIONS[tanFin]);

        tanFin = shiftTan(tanFin,closeFin,fin.getTranslation());
        tanInit = shiftTan(tanInit, closeInit, initial);

        int[] gates = optimzeGates(tanInit, tanFin, closeInit, closeFin);
        
        if(decideCounter(gates[0], gates[1]))
            return bridgeCounter(gates[0], gates[1], initial, fin);
        else
            return bridgeClock(gates[0], gates[1], initial, fin);

        // int diff = Math.abs(tanFin-tanInit);
        // if (tanInit < tanFin)
        // {
        //     if(diff < STATIONS.length/2)
        //     {
        //         System.out.println("small diff, init - fin");

        //         //intersections counter
        //         tanFin = shiftClock(tanFin,closeFin,fin.getTranslation());

        //         tanInit = shiftCounter(tanInit, closeInit, initial);


        //         int[] gates = optimzeGates(tanInit, tanFin, closeInit, closeFin);


        //         return bridgeClock(gates[0], gates[1], initial, fin);

        //     }
        //     else
        //     {
        //         System.out.println("big diff, init - fin");
                
        //         //intersections counter
        //         tanFin = shiftCounter(tanFin,closeFin,fin.getTranslation());

        //         tanInit = shiftClock(tanInit, closeInit, initial);

        //         int[] gates = optimzeGates(tanInit, tanFin, closeInit, closeFin);

        //         return bridgeCounter(gates[0], gates[1], initial, fin);
        //     }
        // }
        // else{
        //     if(tanFin == tanInit)
        //     {
        //         PathPoint[] points = new PathPoint[3];
        //         points[0] = new PathPoint(initial, fin.getRotation());
        //         points[1] = new PathPoint(STATIONS[tanInit].getTranslation(),fin.getRotation());
        //         points[2] = new PathPoint(fin.getTranslation(), fin.getRotation());
        //         return points;
        //     }
        //     else{

        //         if(diff < STATIONS.length/2)
        //         {
        //             System.out.println("small diff, fin - init");

                    
        //             //intersections counter
        //             tanFin = shiftCounter(tanFin,closeFin,fin.getTranslation());

        //             tanInit = shiftClock(tanInit, closeInit, initial);

        //             int[] gates = optimzeGates(tanInit, tanFin, closeInit, closeFin);

        //             return bridgeCounter(gates[0], gates[1], initial, fin);

        //         }
        //         else
        //         {
        //             System.out.println("big diff, fin - init");
        //             int exitStation = tanFin;
        //             int enterStation = tanInit;
                
        //             //intersections counter
        //             tanFin = shiftClock(tanFin,closeFin,fin.getTranslation());

        //             tanInit = shiftCounter(tanInit, closeInit, initial);

        //             int[] gates = optimzeGates(tanInit, tanFin, closeInit, closeFin);

        //             return bridgeClock(gates[0], gates[1], initial, fin);
        //         }
        //     }

           
        // }
    }

    public static void genLineClockwise(Translation2d initial,Pose2d fin,double velocity)
    {

    }

    public static void genLineCounter(Translation2d initial,Pose2d fin,double velocity)
    {
        
    }
}
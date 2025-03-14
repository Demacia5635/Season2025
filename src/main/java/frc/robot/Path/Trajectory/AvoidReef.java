// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Path.Trajectory;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.Path.Utils.Segment;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.utils.LogManager;

/** Add your docs here. */
public class AvoidReef {

    

    public static ArrayList<PathPoint> fixPoints(Translation2d point0, Translation2d point1, Rotation2d wantedAngle) {

        ArrayList<PathPoint> pointsList = new ArrayList<>();

        LogManager.log("LAST POINT: " + point1);
        PathPoint entryPoint = getClosetPoint(point0);
        PathPoint leavePoint = getClosetPoint(point1);

        LogManager.log("ENTRY: " + entryPoint);
        LogManager.log("LEAVE: " + leavePoint);
        int id = findIndex(entryPoint);
        int leaveId = findIndex(leavePoint);
        boolean ascending = isPathAscending(id, leaveId);


        LogManager.log("ENTRY ID: " + id);
        LogManager.log("LEAVE ID: " + leaveId);
        pointsList.add(new PathPoint(point0, new Rotation2d()));
        pointsList.add(entryPoint);

        while (id != leaveId) {
            pointsList.add(FieldTarget.REEF_POINTS[id]);
            id = ascending ? id + 1 : id - 1;
            id = normalize(id);
            
        }
        pointsList.add(new PathPoint(point1, wantedAngle));
        LogManager.log("NEW POINTS: " + pointsList);
        return pointsList;

    }

    private static PathPoint getClosetPoint(Translation2d startingPos) {
        double closetDistance = Integer.MAX_VALUE;
        int index = -1;
        for (int i = 0; i < FieldTarget.REEF_POINTS.length; i++) {
            if (FieldTarget.REEF_POINTS[i].getTranslation().getDistance(startingPos) < closetDistance) {
                LogManager.log("REEF POINT " + i +": "+ (FieldTarget.REEF_POINTS[i]));
                index = i;
                closetDistance = FieldTarget.REEF_POINTS[i].getTranslation().getDistance(startingPos);
            }
        }
        return FieldTarget.REEF_POINTS[index];
    }

    private static int findIndex(PathPoint point) {
        for (int i = 0; i < FieldTarget.REEF_POINTS.length; i++) {
            if (point == FieldTarget.REEF_POINTS[i])
                return i;
        }
        return -1;
    }

    private static boolean isPathAscending(int startid, int endId) {
        int counter = 0;
        int id = startid;
        while (id != endId) {
            counter++;
            id = id + 1;
            id = normalize(id);
        }
        return counter < 3;
    }

    private static int normalize(int id) {
        if (id == -1)
            id = 5;
        if (id == 6)
            id = 0;
        return id;
    }

    public static boolean isGoingThroughReef(Segment segment) {
        for (Segment reefSegment : AutoUtils.REEF_SEGMENTS) {
            boolean intersects = isIntersecting(segment, reefSegment);
            LogManager.log("IS INTERSECTING: " + intersects);
            if (intersects) {
                return true;
            }
        }
        return false;
    }
    
    private static boolean isIntersecting(Segment segment, Segment segmentBase) {
        double x0 = segment.getPoints()[0].getX();
        double y0 = segment.getPoints()[0].getY();
        double x1 = segment.getPoints()[1].getX();
        double y1 = segment.getPoints()[1].getY();
        double x2 = segmentBase.getPoints()[0].getX();
        double y2 = segmentBase.getPoints()[0].getY();
        double x3 = segmentBase.getPoints()[1].getX();
        double y3 = segmentBase.getPoints()[1].getY();
    
        return doIntersect(x0, y0, x1, y1, x2, y2, x3, y3);
    }
    
    private static boolean doIntersect(double x1, double y1, double x2, double y2,
                                       double x3, double y3, double x4, double y4) {
        int d1 = direction(x3, y3, x4, y4, x1, y1);
        int d2 = direction(x3, y3, x4, y4, x2, y2);
        int d3 = direction(x1, y1, x2, y2, x3, y3);
        int d4 = direction(x1, y1, x2, y2, x4, y4);
    
        if (d1 != d2 && d3 != d4) {
            return true;
        }
    
        if (d1 == 0 && onSegment(x3, y3, x4, y4, x1, y1)) return true;
        if (d2 == 0 && onSegment(x3, y3, x4, y4, x2, y2)) return true;
        if (d3 == 0 && onSegment(x1, y1, x2, y2, x3, y3)) return true;
        if (d4 == 0 && onSegment(x1, y1, x2, y2, x4, y4)) return true;
    
        return false;
    }
    
    private static int direction(double xi, double yi, double xj, double yj, double xk, double yk) {
        double val = (yj - yi) * (xk - xj) - (xj - xi) * (yk - yj);
        if (val > 0) return 1;
        if (val < 0) return -1;
        return 0;
    }
    
    private static boolean onSegment(double xi, double yi, double xj, double yj, double xk, double yk) {
        return (xk >= Math.min(xi, xj) && xk <= Math.max(xi, xj)) &&
               (yk >= Math.min(yi, yj) && yk <= Math.max(yi, yj));
    }
    




}


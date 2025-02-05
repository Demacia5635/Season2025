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

/** Add your docs here. */
public class AvoidReef {

    

    public static ArrayList<PathPoint> fixPoints(Translation2d point0, Translation2d point1, Rotation2d wantedAngle) {

        ArrayList<PathPoint> pointsList = new ArrayList<>();

        PathPoint entryPoint = getClosetPoint(point0);
        PathPoint leavePoint = getClosetPoint(point1);

        int id = findIndex(entryPoint);
        int leaveId = findIndex(leavePoint);
        boolean ascending = isPathAscending(id, leaveId);

        pointsList.add(new PathPoint(point0, new Rotation2d()));

        while (id != leaveId) {
            pointsList.add(FieldTarget.REEF_POINTS[id]);
            id = ascending ? id + 1 : id - 1;
            id = normalize(id);
        }

        pointsList.add(new PathPoint(point1, wantedAngle));
        return pointsList;

    }

    private static PathPoint getClosetPoint(Translation2d startingPos) {
        double closetDistance = Integer.MAX_VALUE;
        int index = -1;
        for (int i = 0; i < FieldTarget.REEF_POINTS.length; i++) {
            if (FieldTarget.REEF_POINTS[i].getTranslation().getDistance(startingPos) < closetDistance) {
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
        for (int i = 0; i < AutoUtils.REEF_SEGMENTS.length; i++) {
            if (isIntersecting(segment, Math.sqrt(2) / 2, AutoUtils.REEF_SEGMENTS[i])) {
                return true;
            }
        }
        return false;
    }

    private static boolean isIntersecting(Segment segment, double segmentWidth, Segment segmentBase) {
        double x0 = segment.getPoints()[0].getX();
        double y0 = segment.getPoints()[0].getY();
        double x1 = segment.getPoints()[1].getX();
        double y1 = segment.getPoints()[1].getY();
        double x2 = segmentBase.getPoints()[0].getX();
        double y2 = segmentBase.getPoints()[0].getY();
        double x3 = segmentBase.getPoints()[1].getX();
        double y3 = segmentBase.getPoints()[1].getY();

        double m1 = (y0 - y1) / (x0 - x1);
        double m2 = (y2 - y3) / (x2 - x3);

        if (m1 == m2) {
            return false;
        }

        double x = (m2 * x2 - m1 * x1 + y1 - y2) / (m2 - m1);
        double y = m2 * x - m2 * x2 + y2;

        boolean withinSegment1 = (x >= Math.min(x0, x1) && x <= Math.max(x0, x1)) &&
                (y >= Math.min(y0, y1) && y <= Math.max(y0, y1));

        boolean withinSegment2 = (x >= Math.min(x2, x3) && x <= Math.max(x2, x3)) &&
                (y >= Math.min(y2, y3) && y <= Math.max(y2, y3));

        return withinSegment1 && withinSegment2;
    }




}


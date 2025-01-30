package frc.robot.PathFollow;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LogManager;
import edu.wpi.first.math.trajectory.Trajectory.State;

import static frc.robot.chassis.commands.auto.AutoUtils.REEF_SEGMENTS;
import static frc.robot.chassis.commands.auto.AutoUtils.fieldElements;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.chassis.commands.auto.AlignToTag;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.TrapezoidNoam;

public class PathFollow extends Command {
  Timer timer = new Timer();
  double maxVel;

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d closestAprilTag = new Pose2d();

  Pose2d chassisPose = new Pose2d();
  double distanceOffset = 0.01;
  double pathLength;

  double totalLeft;
  int segmentIndex = 0;

  ArrayList<Segment> segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;

  TrapezoidNoam driveTrapezoid;
  TrapezoidNoam rotationTrapezoid;
  Field2d trajField = new Field2d();

  double driveVelocity = 0;
  double rotationVelocity = 0;
  public static double fieldLength = 17.524437; // in meters
  public static double fieldHeight = 8.109847; // in meters
  boolean isRed;
  boolean rotateToSpeaker = false;

  Trajectory traj;
  double distancePassed = 0;
  PathPoint[] points;
  double finishVel;

  boolean autoRotate = false;
  double autoRotateVel = 2;
  Pose2d[] aprilTagsPositions = new Pose2d[]{new Pose2d()};
  Rotation2d finalAngle;
  boolean isConstVel = false;
  boolean isPrecise = false;
  double reefRadius = 3;
  FIELD_POSITION toGoElement;
  AlignToTag alignToTag;
  double minVel = 0.5;

  public PathFollow(PathPoint[] points, double velocity) {
    this(RobotContainer.robotContainer.chassis, points, velocity, velocity * 2,
        0, RobotContainer.isRed());
    this.maxVel = velocity;
  }

  public PathFollow(PathPoint[] points, Rotation2d finalAngle, double maxVel, boolean isConstVel, boolean isPrecise) {
    this(RobotContainer.robotContainer.chassis, points, maxVel,
        8,
        0, RobotContainer.isRed());
    this.finalAngle = finalAngle;
    this.maxVel = maxVel;
    this.isConstVel = isConstVel;
    this.isPrecise = isPrecise;
  }

  public PathFollow(PathPoint[] points, Rotation2d finalAngle, double maxVel, boolean isConstVel, boolean isPrecise, FIELD_POSITION toGoElement, AlignToTag alignToTag) {
    this(RobotContainer.robotContainer.chassis, points, maxVel,
        8,
        0, RobotContainer.isRed());
    this.finalAngle = finalAngle;
    this.maxVel = maxVel;
    this.isConstVel = isConstVel;
    this.isPrecise = isPrecise;
    this.toGoElement = toGoElement;
    this.alignToTag = alignToTag;
  }

  public PathFollow(Chassis chassis, PathPoint[] points, double maxVel, double maxAcc, double finishVel) {
    this.points = points;
    this.finishVel = finishVel;
    this.chassis = chassis;
    addRequirements(chassis);
    
    driveTrapezoid = new TrapezoidNoam(maxVel, maxAcc);
    rotationTrapezoid = new TrapezoidNoam(180, 360);
    
    segments = new ArrayList<Segment>();
  }

  public PathFollow(Chassis chassis, PathPoint[] points, double maxVel, double maxAcc, double finishVel,
      boolean rotateToSpeaker) {
    this(chassis, points, maxVel, maxAcc, finishVel);
    this.rotateToSpeaker = rotateToSpeaker;
  }

  private boolean isIntersecting(Segment segment, double segmentWidth, Segment segmentBase) {
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

  private boolean isBumpingReef(Segment segment) {
    for (int i = 0; i < REEF_SEGMENTS.length; i++) {
      if (isIntersecting(segment, Math.sqrt(2) / 2, REEF_SEGMENTS[i])) {
        return true;
      }
    }
    return false;
  }

  private boolean isPathAscending(int startid, int endId) {
    int counter = 0;
    int id = startid;
    while (id != endId) {
      counter++;
      id = id + 1;
      id = normalis(id);
    }
    return counter < 3;
  }

  private int normalis(int id) {
    if (id == -1)
      id = 5;
    if (id == 6)
      id = 0;
    return id;
  }

  private void evasion(Segment segment) {
    ArrayList<PathPoint> pointsList = new ArrayList<>();

    PathPoint entryPoint = getClosetPoint(segment.getPoints()[0]);
    PathPoint leavePoint = getClosetPoint(segment.getPoints()[1]);

    int id = findIndex(entryPoint);
    int leaveId = findIndex(leavePoint);
    boolean ascending = isPathAscending(id, leaveId);

    pointsList.add(new PathPoint(segment.getPoints()[0], segment.getPoints()[0].getAngle()));

    while (id != leaveId) {
      pointsList.add(AutoUtils.REEF_POINTS[id]);
      id = ascending ? id + 1 : id - 1;
      id = normalis(id);
    }

    pointsList.add(new PathPoint(segment.getPoints()[1], fieldElements.get(toGoElement).getRotation()));

    // Update current path instead of creating new PathFollow
    this.points = pointsList.toArray(new PathPoint[0]);
    
    // Reset segments
    segments.clear();
    
    // Reinitialize path with new points
    if (points.length < 3) {
      Segment cur = new Leg(points[0].getTranslation(), points[1].getTranslation(), points[1].isAprilTag());
      segments.add(cur);
    } else {
      corners = new RoundedPoint[points.length - 2];
      for (int i = 0; i < points.length - 2; i++) {
        corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2], points[i].isAprilTag());
      }

      segments.add(0, corners[0].getAtoCurveLeg());

      for (int i = 0; i < corners.length - 1; i += 1) {
        segments.add(corners[i].getArc());
        segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart(),
            points[i].isAprilTag()));
      }

      segments.add(corners[corners.length - 1].getArc());
      segments.add(corners[corners.length - 1].getCtoCurveLeg());
    }

    // Recalculate path length
    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;
    segmentIndex = 0;
  }

  private int findIndex(PathPoint point) {
    for (int i = 0; i < AutoUtils.REEF_POINTS.length; i++) {
      if (point == AutoUtils.REEF_POINTS[i])
        return i;
    }
    return -1;
  }

  private PathPoint getClosetPoint(Translation2d startingPos) {
    double closetDistance = Integer.MAX_VALUE;
    int index = -1;
    for (int i = 0; i < AutoUtils.REEF_POINTS.length; i++) {
      if (AutoUtils.REEF_POINTS[i].getTranslation().getDistance(startingPos) < closetDistance) {
        index = i;
        closetDistance = AutoUtils.REEF_POINTS[i].getTranslation().getDistance(startingPos);
      }
    }
    return AutoUtils.REEF_POINTS[index];
  }

  @Override
  public void initialize() {
    isRed = false;
    points[0] = new PathPoint(chassis.getPose().getX(), chassis.getPose().getY(), chassis.getPose().getRotation(),
        points[0].getRadius(), false);

    if (isRed) {
      points[0] = new PathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
          Rotation2d.fromDegrees(180).minus(chassis.getPose().getRotation()), points[0].getRadius(), false);
      for (int i = 1; i < points.length; i++) {
        points[i] = new PathPoint(fieldLength - points[i].getX(), fieldHeight - points[i].getY(),
            Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
            points[i].getRadius(), points[i].isAprilTag());
      }
    }

    // Initialize path
    initializePath();

    List<State> list = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      State temp = new State();
      temp.poseMeters = new Pose2d(points[i].getX(), points[i].getY(), new Rotation2d(0));
      list.add(temp);
    }

    traj = new Trajectory(list);
    trajField.getObject("TrajTEST").setTrajectory(traj);

    vecVel = new Translation2d(0, 0);
  }

  private void initializePath() {
    corners = new RoundedPoint[points.length - 2];
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2], points[i].isAprilTag());
    }

    if (points.length < 3) {
      Segment cur = new Leg(points[0].getTranslation(), points[1].getTranslation(), points[1].isAprilTag());
      if (isBumpingReef(cur)) {
        evasion(cur);
        return;
      }
      segments.add(cur);
    } else {
      segments.add(0, corners[0].getAtoCurveLeg());

      for (int i = 0; i < corners.length - 1; i += 1) {
        segments.add(corners[i].getArc());
        segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart(),
            points[i].isAprilTag()));
      }
      segments.add(corners[corners.length - 1].getArc());
      segments.add(corners[corners.length - 1].getCtoCurveLeg());
    }

    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;
    segmentIndex = 0;
  }

  @Override
  public void execute() {

    trajField.setRobotPose(chassis.getPose());

    chassisPose = chassis.getPose();
  

    distancePassed = totalLeft - segments.get(segmentIndex).distancePassed(chassisPose.getTranslation());

    if (segments.get(segmentIndex).distancePassed(chassisPose.getTranslation()) >= segments.get(segmentIndex).getLength()
        - distanceOffset) {
      totalLeft -= segments.get(segmentIndex).getLength();
      if (segmentIndex != segments.size() - 1)
        segmentIndex++;
    }
    /*driveVelocity = driveTrapezoid.calculate(
      totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
        currentVelocity.getNorm(), finishVel);*/

    driveVelocity = isConstVel ? maxVel : Math.min((totalLeft - segments.get(segmentIndex).distancePassed(chassis.getPose().getTranslation())) * 3, maxVel) ;
    driveVelocity = Math.max(driveVelocity, minVel);
    
    Translation2d velVector = segments.get(segmentIndex).calc(chassisPose.getTranslation(), driveVelocity);

   

    if (totalLeft <= 0.1) velVector = new Translation2d(0, 0);
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), 0);
    
    
    
      chassis.setVelocitiesRotateToAngle(speed, finalAngle);
  

  }

  public PathFollow setAutoRotate(double rate) {
    autoRotate = true;
    autoRotateVel = rate;
    return this;
  }

  @Override
  public void end(boolean interrupted) {
    if(finishVel == 0) chassis.setVelocities(new ChassisSpeeds(0,0,0));
    driveTrapezoid.debug = false;
    LogManager.log("ended");
    // .useAcceleration = true;
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= (isPrecise ? 0.1 : 0.25);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addStringProperty("Current Segment", () -> currentSegmentInfo(),
    // null);
    super.initSendable(builder);
    builder.addDoubleProperty("Distance Passed", () -> {
      return distancePassed;
    }, null);
    builder.addDoubleProperty("Total Left", () -> {
      return totalLeft;
    }, null);
    builder.addDoubleProperty("Velocity", () -> {
      return driveVelocity;
    }, null);
    builder.addDoubleProperty("Rotation Velocity", () -> {
      return Math.toDegrees(rotationVelocity);
    }, null);
    builder.addDoubleProperty("Angle", () -> {
      return chassisPose.getRotation().getDegrees();
    }, null);
    builder.addDoubleProperty("Pose X", () -> chassis.getPose().getX(), null);
    builder.addDoubleProperty("Pose Y", () -> chassis.getPose().getY(), null);
  }

  public void printSegments() {
    for (Segment s : segments) {
      LogManager.log(s);
    }
  }
}
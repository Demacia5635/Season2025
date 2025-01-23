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


import edu.wpi.first.math.trajectory.Trajectory.State;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.chassis.commands.auto.AlignToTag;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_ELEMENTS;
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
  Translation2d reefCenter = new Translation2d(fieldLength-4.5,fieldHeight-4); 
  FIELD_ELEMENTS toGoElement;
  boolean stop = false;
  AlignToTag alignToTag;
  
  /**
   * Creates a new path follower using the given points.
   * 
   * @param chassis
   * @param points   from blue alliance
   * @param maxVel   the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */

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

  public PathFollow(PathPoint[] points, Rotation2d finalAngle, double maxVel, boolean isConstVel, boolean isPrecise, FIELD_ELEMENTS toGoElement, AlignToTag alignToTag) {
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

    // gets the wanted angle for the robot to finish the path in

    // creates new coreners array of the "arc points" in the path
    addRequirements(chassis);

    // creates trapezoid object for drive and rotation
    driveTrapezoid = new TrapezoidNoam(maxVel, maxAcc);
    rotationTrapezoid = new TrapezoidNoam(180, 360);

    // calculate the total length of the path
    segments = new ArrayList<Segment>();

  }

  public PathFollow(Chassis chassis, PathPoint[] points, double maxVel, double maxAcc, double finishVel,
      boolean rotateToSpeaker) {
    this(chassis, points, maxVel, maxAcc, finishVel);
    this.rotateToSpeaker = rotateToSpeaker;
  }
  

  private boolean isIntersectingSegment(Segment segment){
    // Extract coordinates of the segment
    Translation2d segmentVector = segment.getPoints()[0].minus(segment.getPoints()[1]);
    Translation2d startToCenter = reefCenter.minus(segment.getPoints()[0]);
    Translation2d endToCenter = reefCenter.minus(segment.getPoints()[1]);

    if (startToCenter.getNorm() > segmentVector.getNorm()) {
      return false;
    }
    if (endToCenter.getNorm() > segmentVector.getNorm()) {
      return false;
    }

    Translation2d vectorA = new Translation2d(reefRadius, startToCenter.getAngle().plus(Rotation2d.fromDegrees(90)));
    //Translation2d vectorB = new Translation2d(reefRadius, startToCenter.getAngle().plus(Rotation2d.fromDegrees(90))); // may need vector B 
    Rotation2d highBound = startToCenter.plus(vectorA).getAngle();
    Rotation2d segmentAngle = segmentVector.getAngle().minus(startToCenter.getAngle());
    
    return Math.abs(segmentAngle.getRadians()) < Math.abs(highBound.getRadians());
  }


  private Translation2d getShortenPoint(Segment segment){
    double x0 = segment.getPoints()[0].getX();
    double y0 = segment.getPoints()[0].getY();
    double x1 = segment.getPoints()[1].getX();
    double y1 = segment.getPoints()[1].getY();
    
    double m = (y1 - y0) / (x1 - x0);
    
    double opM = -1 / m;
    
    double x1Intersection = (x0 + x1) / 2 + 1;
    double y1Intersection = opM * (x1Intersection - (x0 + x1) / 2) + (y0 + y1) / 2;
    
    double x2Intersection = (x0 + x1) / 2 - 1;
    double y2Intersection = opM * (x2Intersection - (x0 + x1) / 2) + (y0 + y1) / 2;

    Translation2d intersection1 = new Translation2d(x1Intersection, y1Intersection);
    Translation2d intersection2 = new Translation2d(x2Intersection, y2Intersection);

    Translation2d centerToIntersection1 = intersection1.minus(reefCenter);
    Translation2d centerToIntersection2 = intersection2.minus(reefCenter);

    Translation2d centerToIntersection = intersection1.getNorm() > intersection2.getNorm() ? centerToIntersection2 : centerToIntersection1;

    return reefCenter.plus(centerToIntersection);
  }
  /*private boolean getClosetPoint(Pose2d startingPose){
    double closetDistance = Integer.MAX_VALUE;
    for(int i = 0; i < AutoUtils.fieldElements.size(); i++){
      
    }
  }*/

  /*
   * public String currentSegmentInfo() {
   * 
   * if (segments == null)
   * return "";
   * return segments[segmentIndex].toString();
   * }
   */

  @Override
  public void initialize() {
    
    isRed = false;//RobotContainer.isRed();
    // sets first point to chassis pose to prevent bugs with red and blue alliance
    points[0] = new PathPoint(chassis.getPose().getX(), chassis.getPose().getY(), chassis.getPose().getRotation(),
        points[0].getRadius(), false);

    // case for red alliance (blue is the default)
    if (isRed) {
      reefCenter = new Translation2d(fieldLength - reefCenter.getX(), fieldHeight - reefCenter.getY());

      points[0] = new PathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
          Rotation2d.fromDegrees(180).minus(chassis.getPose().getRotation()), points[0].getRadius(), false);
      for (int i = 1; i < points.length; i++) {
        points[i] = new PathPoint(fieldLength - points[i].getX(), fieldHeight - points[i].getY(),
            Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
            points[i].getRadius(), points[i].isAprilTag());
      }
    }
    corners = new RoundedPoint[points.length - 2];
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2], points[i].isAprilTag());
    }
    
    System.out.println();

    if (points.length < 3) {
      Segment cur = new Leg(points[0].getTranslation(), points[1].getTranslation(), points[1].isAprilTag());
      System.out.println(isIntersectingSegment(cur));
      if(isIntersectingSegment(cur)){
        new PathFollow(
          new PathPoint[]{new PathPoint(new Translation2d(), new Rotation2d()),
          new PathPoint(getShortenPoint(cur), new Rotation2d()),
            points[points.length - 1]}, points[points.length -1].getRotation(),
             2, false, true).andThen(alignToTag).schedule();
        stop = true;
      }
      segments.add(cur);
      
    }
    // case for more then 1 segment
    else {
      // creates the first leg
      segments.add(0, corners[0].getAtoCurveLeg());

      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i += 1) {

        segments.add(corners[i].getArc());

        segments.add(new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart(),
            points[i].isAprilTag()));
      }
      // creates the last arc and leg
      segments.add(corners[corners.length - 1].getArc());
      segments.add(corners[corners.length - 1].getCtoCurveLeg());
    }

    // calculates the length of the entire path
    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;
    segmentIndex = 0;

    List<State> list = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      State temp = new State();
      temp.poseMeters = new Pose2d(points[i].getX(), points[i].getY(), new Rotation2d(0));
      list.add(temp);
    }

    // System.out.println("LIST: " + list);

    traj = new Trajectory(list);
    trajField.getObject("TrajTEST").setTrajectory(traj);

    vecVel = new Translation2d(0, 0);
  }

  // calculates the position of the closet april tag and returns it's position
  boolean foundAprilTag = false;

  public Rotation2d getAngleApriltag() {
    Translation2d finalVector = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    // checks the distance from each april tag and finds
    for (int i = 0; i < aprilTagsPositions.length; i++) {

      Translation2d currentAprilTagVector = chassis.getPose().minus(aprilTagsPositions[i]).getTranslation();

      if (currentAprilTagVector.getNorm() < finalVector.getNorm()) {
        finalVector = currentAprilTagVector;
      }

    }
    foundAprilTag = true;

    return finalVector.getAngle();
  }

  public static double convertAlliance(double x) {
    return fieldLength - x;
  }

  public static double fixY(double y) {
    return fieldHeight - y;
  }

  @Override
  public void execute() {
    if(stop) return;

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

    Translation2d velVector = segments.get(segmentIndex).calc(chassisPose.getTranslation(), driveVelocity);

   

    if (totalLeft <= 0.1)
      velVector = new Translation2d(0, 0);
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
    // .useAcceleration = true;
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= (isPrecise ? 0.1 : 0.6) || stop;
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
     System.out.println(s);
    }
  }
}
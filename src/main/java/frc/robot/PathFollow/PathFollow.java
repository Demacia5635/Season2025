package frc.robot.PathFollow;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.LogManager;


import edu.wpi.first.math.trajectory.Trajectory.State;

import static frc.robot.chassis.commands.auto.AutoUtils.REEF_SEGMENTS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.objdetect.BarcodeDetector;

import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.PathPoint;
import frc.robot.chassis.commands.auto.AlignToTag;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.AutoUtils.FIELD_POSITION;
import frc.robot.chassis.commands.auto.AutoUtils.REEF_SEGMENTS;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
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
  boolean stop = false;
  AlignToTag alignToTag;
  boolean isFirstTry;
  double minVel = 0.5;
  
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
    this.stop = false;
  
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

  private boolean isIntersecting (Segment segment1, Segment segment2){
    double x0 = segment1.getPoints()[0].getX();
    double y0 = segment1.getPoints()[0].getY();
    double x1 = segment1.getPoints()[1].getX();
    double y1 = segment1.getPoints()[1].getY();
    double x2 = segment2.getPoints()[0].getX();
    double y2 = segment2.getPoints()[0].getY();
    double x3 = segment2.getPoints()[1].getX();
    double y3 = segment2.getPoints()[1].getY();

    double m1 = (y0 - y1) / (x0 - x1);
    double m2 = (y2 - y3) / (x2 - x3);

    if (m1 == m2){
      return false;
    }

    double x = (m2*x2 - m1*x1 + y1 - y2) / (m2 - m1);
    double y = m2*x - m2*x2 + y2;

    boolean withinSegment1 = (x >= Math.min(x0, x1) && x <= Math.max(x0, x1)) &&
                             (y >= Math.min(y0, y1) && y <= Math.max(y0, y1));

    boolean withinSegment2 = (x >= Math.min(x2, x3) && x <= Math.max(x2, x3)) &&
                             (y >= Math.min(y2, y3) && y <= Math.max(y2, y3));

    return withinSegment1 && withinSegment2;
  }
  private boolean isBumpingReef(Segment segment){
    for (int i = 0; i < REEF_SEGMENTS.length; i++){
      if(isIntersecting(segment, REEF_SEGMENTS[i])){
        return true;
      }
    }
    return false;
    }
    private boolean isPathAscending(int startid, int endId){
    int counter = 0;
    int id = startid;
    while (id != endId) {
      counter++;
      id = id + 1;
      if (id == 6) id = 0;
    }
    return counter < 3;
    }

    private void evasion(Segment segment) {
    ArrayList<PathPoint> pointsList = new ArrayList<>();

    PathPoint entryPoint = getClosetPoint(segment.getPoints()[0]);
    PathPoint leavePoint = getClosetPoint(segment.getPoints()[1]);

    int id = findIndex(entryPoint);
    int leaveId = findIndex(leavePoint);
    boolean ascending = isPathAscending(id, leaveId);

    while (id != leaveId) {
      if (id == -1) id = 5;
      if (id == 6) id = 0;
      pointsList.add(AutoUtils.REEF_POINTS[id]);
      id = ascending ? id + 1 : id - 1;
    }

    pointsList.add(leavePoint);

    PathPoint[] pathPoints = new PathPoint[pointsList.size()];
    for (int i = 0; i < pathPoints.length; i++) {
        pathPoints[i] = pointsList.get(i);
    }

    new PathFollow(pathPoints, points[points.length - 1].getRotation(),
     3, false, true)
     .alongWith(new InstantCommand(()->RobotContainer.arm.setState(ARM_ANGLE_STATES.STARTING)))
     .andThen(alignToTag).schedule();
  }
  
  private void getPathPoint(Segment segment){
    ArrayList<PathPoint> pointsList = new ArrayList<>();

    PathPoint entryPoint = getClosetPoint(segment.getPoints()[0]);
    PathPoint leavePoint = getClosetPoint(segment.getPoints()[1]);
    
      boolean isWithIndexs = Math.abs(findIndex(entryPoint) - findIndex(leavePoint))
       > Math.abs(findIndex(leavePoint) - findIndex(entryPoint));
       boolean isEntrySmaller = findIndex(leavePoint) > findIndex(entryPoint);

      pointsList.add(entryPoint);
      if(isWithIndexs){
        if(isEntrySmaller){
          for(int j = findIndex(entryPoint); j < findIndex(leavePoint); j++){
            pointsList.add(AutoUtils.REEF_POINTS[j]);
          }
        }
        else{
          
          for(int j = 0; j < Math.abs(findIndex(entryPoint) - findIndex(leavePoint) - 1); j++){
            int curIndex = findIndex(entryPoint) + j + 1;
            if(curIndex == 6) curIndex = 0;
            pointsList.add(AutoUtils.REEF_POINTS[curIndex]);
          }
          
        }
      }
      else{
        if(!isEntrySmaller){
          
          for(int j = findIndex(entryPoint) - 1; j > findIndex(leavePoint); j--){
            if(j == -1) j = 5;
            pointsList.add(AutoUtils.REEF_POINTS[j]);
          }
        }
        else{
          for(int j = findIndex(entryPoint); j < findIndex(leavePoint); j--){
            if(j == -1) j = 5;
            pointsList.add(AutoUtils.REEF_POINTS[j]);
          }
        }

      }
      pointsList.add(new PathPoint(segment.getPoints()[1], new Rotation2d()));
      
    
    PathPoint[] pathPoints = new PathPoint[pointsList.size()];
    for(int i = 0; i < pathPoints.length; i++){
      pathPoints[i] = pointsList.get(i);
    }

    new PathFollow(pathPoints, points[points.length-1].getRotation(), 3.5, false, true).andThen(alignToTag).schedule();
  }
  
  private int findIndex(PathPoint point){
    
    for(int i = 0; i < AutoUtils.REEF_POINTS.length; i++){
      if(point == AutoUtils.REEF_POINTS[i]) return i;
    }
    return -1;
  }

  private boolean getIsGoUp(int startId, int endId){
    int counter = 0;
    for(int i = 0; i < 5; i++){
      if(startId != endId){
        counter++;
      }
      else{
        break;
      }
      startId ++;
      if (startId == 6) startId = 0;
    }
    return counter < 3;
  }
  private PathPoint getClosetPoint(Translation2d startingPos){
    double closetDistance = Integer.MAX_VALUE;
    int index = -1;
    for(int i = 0; i < AutoUtils.REEF_POINTS.length; i++){
      if(AutoUtils.REEF_POINTS[i].getTranslation().getDistance(startingPos) < closetDistance){
        index = i;
        closetDistance = AutoUtils.REEF_POINTS[i].getTranslation().getDistance(startingPos);
      }
    }
    return AutoUtils.REEF_POINTS[index];
  }

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
      if(isBumpingReef(cur)){
        //getPathPoint(cur);
        evasion(cur);
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
    // .useAcceleration = true;
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= (isPrecise ? 0.1 : 0.6);
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
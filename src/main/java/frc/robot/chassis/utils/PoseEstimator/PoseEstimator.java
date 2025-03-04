// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils.PoseEstimator;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class PoseEstimator {
  private final DemaciaOdometry odometry;
  private double[] visionSTD = new double[3];

  private double MAX_STD = 1.5;

  private static final double kBufferDuration = 1.5;
  // Maps timestamps to odometry-only pose estimates
  private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer = TimeInterpolatableBuffer
      .createBuffer(kBufferDuration);
  // Maps timestamps to vision updates
  // Always contains one entry before the oldest entry in m_odometryPoseBuffer,
  // unless there have
  // been no vision measurements after the last reset
  private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();
  private Pose2d poseEstimation;

  public PoseEstimator(
      DemaciaOdometry odometry,
      double[] visionSTD) {

    this.odometry = odometry;
    setVisionSTD(visionSTD);

    this.poseEstimation = odometry.getEstimatedPosition();

  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to
   * change trust in
   * vision measurements after the autonomous period, or to change trust as
   * distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision
   *                                 measurements. Increase these
   *                                 numbers to trust global measurements from
   *                                 vision less. This matrix is in the form [x,
   *                                 y,
   *                                 theta]ᵀ, with units in meters and radians.
   */
  public void setVisionSTD(double[] visionMeasurementStdDevs) {
    this.visionSTD = visionMeasurementStdDevs;
  }

  

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The pose's timestamp in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is
   *         empty).
   */
  public Optional<Pose2d> sampleAt(double timestampSeconds) {
    // Step 0: If there are no odometry updates to sample, skip.
    if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    }

    // Step 1: Make sure timestamp matches the sample from the odometry pose buffer.
    // (When sampling,
    // the buffer will always use a timestamp between the first and last timestamps)
    double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();
    double newestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().lastKey();
    timestampSeconds = MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

    // Step 2: If there are no applicable vision updates, use the odometry-only
    // information.
    if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
      return m_odometryPoseBuffer.getSample(timestampSeconds);
    }

    // Step 3: Get the latest vision update from before or at the timestamp to
    // sample at.
    double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
    var visionUpdate = m_visionUpdates.get(floorTimestamp);

    // Step 4: Get the pose measured by odometry at the time of the sample.
    var odometryEstimate = m_odometryPoseBuffer.getSample(timestampSeconds);

    // Step 5: Apply the vision compensation to the odometry pose.
    return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
  }

  /** Removes stale vision updates that won't affect sampling. */
  private void cleanUpVisionUpdates() {
    // Step 0: If there are no odometry samples, skip.
    if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return;
    }

    // Step 1: Find the oldest timestamp that needs a vision update.
    double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();

    // Step 2: If there are no vision updates before that timestamp, skip.
    if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
      return;
    }

    // Step 3: Find the newest vision update timestamp before or at the oldest
    // timestamp.
    double newestNeededVisionUpdateTimestamp = m_visionUpdates.floorKey(oldestOdometryTimestamp);

    // Step 4: Remove all entries strictly before the newest timestamp we need.
    m_visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
    if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
        || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
            > timestampSeconds) {
      return;
    }

    // Step 1: Clean up any old entries
    cleanUpVisionUpdates();

    // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
    var odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds);

    if (odometrySample.isEmpty()) {
      return;
    }

    // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
    // made.
    var visionSample = sampleAt(timestampSeconds);

    if (visionSample.isEmpty()) {
      return;
    }

    
    // Step 4: Measure the twist between the old pose estimate and the vision pose.
    Twist2d twist = visionSample.get().log(visionRobotPoseMeters);
    double[] visionWeights = getWeights(visionSTD);
    Twist2d scaledTwist = new Twist2d(twist.dx * visionWeights[0], twist.dy * visionWeights[1], twist.dtheta * visionWeights[2]);


    // Step 7: Calculate and record the vision update.
    var visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
    
    m_visionUpdates.put(timestampSeconds, visionUpdate);

    // Step 8: Remove later vision measurements. (Matches previous behavior)
    m_visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

    // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
    // it's guaranteed to be the latest vision update.
    poseEstimation = visionUpdate.compensate(odometry.getEstimatedPosition());
  }

  
  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The current encoder readings.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, wheelPositions);
  }


  public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    var odometryEstimate = odometry.update(gyroAngle, wheelPositions);

    m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);

    if (m_visionUpdates.isEmpty()) {
      poseEstimation = odometryEstimate;
    } else {
      var visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
      poseEstimation = visionUpdate.compensate(odometryEstimate);
    }

    return getEstimatedPosition();
  }


  private double calculateWeight(double stdDev) 
  {
    if(stdDev > MAX_STD) return 0;
    // Define a scaling factor to adjust sensitivity
    double scale = 1.0;

    // Compute the weight using an exponential decay function
    return MathUtil.clamp(Math.exp(-stdDev * scale), 0, 1);
  }

  private double[] getWeights(double[] stdDevs){
    double[] result = new double[3];
    for (int i = 0; i < result.length; i++) {
      result[i] = calculateWeight(stdDevs[i]);
    }
    return result;
  }

  public Pose2d getEstimatedPosition() {
    return poseEstimation;
  }














  
  private static final class VisionUpdate {
    // The vision-compensated pose estimate.
    private final Pose2d visionPose;

    // The pose estimated based solely on odometry.
    private final Pose2d odometryPose;

    /**
     * Constructs a vision update record with the specified parameters.
     *
     * @param visionPose   The vision-compensated pose estimate.
     * @param odometryPose The pose estimate based solely on odometry.
     */
    private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
      this.visionPose = visionPose;
      this.odometryPose = odometryPose;
    }

    /**
     * Returns the vision-compensated version of the pose. Specifically, changes the
     * pose from being
     * relative to this record's odometry pose to being relative to this record's
     * vision pose.
     *
     * @param pose The pose to compensate.
     * @return The compensated pose.
     */
    public Pose2d compensate(Pose2d pose) {
      var delta = pose.minus(this.odometryPose);
      return this.visionPose.plus(delta);
    }
  }

}

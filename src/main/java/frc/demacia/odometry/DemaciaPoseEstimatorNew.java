// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.odometry;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class DemaciaPoseEstimatorNew {
    private final double MAX_HISTORY_SECONDS = 1.5;

    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());
    private DemaciaOdometry odometry;

    private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer = TimeInterpolatableBuffer
            .createBuffer(MAX_HISTORY_SECONDS);

    private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

    private Pose2d poseEstimate;

    public DemaciaPoseEstimatorNew(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD, Matrix<N3, N1> visionSTD) {
        this.odometry = new DemaciaOdometry(modulePositions);
        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateSTD.get(i, 0) * stateSTD.get(i, 0));
        }
        updateVisionSTD(visionSTD);
        this.poseEstimate = Pose2d.kZero;
    }

    public void updateVisionSTD(Matrix<N3, N1> visionSTD) {
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionSTD.get(i, 0) * visionSTD.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>
     * The gyroscope angle does not need to be reset here on the user's robot code.
     * The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param gyroAngle      The angle reported by the gyroscope.
     * @param wheelPositions The current encoder readings.
     * @param poseMeters     The position on the field that your robot is at.
     */
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        // Reset state estimate and error covariance
        odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
        odometryPoseBuffer.clear();
        visionUpdates.clear();
        poseEstimate = odometry.getPose2d();
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
        odometryPoseBuffer.clear();
        visionUpdates.clear();
        poseEstimate = odometry.getPose2d();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimate;
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
        if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer.
        // (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds = MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only
        // information.
        if (visionUpdates.isEmpty() || timestampSeconds < visionUpdates.firstKey()) {
            return odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to
        // sample at.
        double floorTimestamp = visionUpdates.floorKey(timestampSeconds);
        var visionUpdate = visionUpdates.get(floorTimestamp);

        // Step 4: Get the pose measured by odometry at the time of the sample.
        var odometryEstimate = odometryPoseBuffer.getSample(timestampSeconds);
        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
    }

    /** Removes stale vision updates that won't affect sampling. */
    private void cleanUpVisionUpdates() {
        // Step 0: If there are no odometry samples, skip.
        if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest
        // timestamp.
        double newestNeededVisionUpdateTimestamp = visionUpdates.floorKey(oldestOdometryTimestamp);

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's
        // timespan, skip.
        if (odometryPoseBuffer.getInternalBuffer().isEmpty()
                || odometryPoseBuffer.getInternalBuffer().lastKey() - MAX_HISTORY_SECONDS > timestampSeconds) {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision
        // measurement was made.
        var odometrySample = odometryPoseBuffer.getSample(timestampSeconds);

        if (odometrySample.isEmpty()) {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision
        // measurement was
        // made.
        var visionSample = sampleAt(timestampSeconds);

        if (visionSample.isEmpty()) {
            return;
        }

        // Step 4: Measure the transform between the old pose estimate and the vision
        // pose.
        var transform = visionRobotPoseMeters.minus(visionSample.get());

        // Step 5: We should not trust the transform entirely, so instead we scale this
        // transform by a
        // Kalman
        // gain matrix representing how much we trust vision measurements compared to
        // our current pose.
        var k_times_transform = m_visionK.times(
                VecBuilder.fill(
                        transform.getX(), transform.getY(), transform.getRotation().getRadians()));

        // Step 6: Convert back to Transform2d.
        var scaledTransform = new Transform2d(
                k_times_transform.get(0, 0),
                k_times_transform.get(1, 0),
                Rotation2d.fromRadians(k_times_transform.get(2, 0)));

        // Step 7: Calculate and record the vision update.
        var visionUpdate = new VisionUpdate(visionSample.get().plus(scaledTransform), odometrySample.get());
        visionUpdates.put(timestampSeconds, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

        // Step 9: Update latest pose estimate. Since we cleared all updates after this
        // vision update,
        // it's guaranteed to be the latest vision update.
        poseEstimate = visionUpdate.compensate(odometry.getPose2d());
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        updateVisionSTD(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void addOdometryCalculation(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, double timestampSeconds) {
        var odometryEstimation = odometry.update(gyroAngle, wheelPositions);
        odometryPoseBuffer.addSample(timestampSeconds, odometryEstimation);

        if (visionUpdates.isEmpty()) {
            poseEstimate = odometryEstimation;
        } else {
            var latestVisionUpdate = visionUpdates.get(visionUpdates.lastKey());
            poseEstimate = latestVisionUpdate.compensate(odometryEstimation);
        }
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

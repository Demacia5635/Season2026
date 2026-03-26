// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.Trgectory;

import java.util.ArrayList;
import java.util.logging.LogManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import frc.demacia.path.Trgectory.TrajectoryConstants.PathsConstraints;
import frc.demacia.path.utils.Arc;
import frc.demacia.path.utils.Leg;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.path.utils.RoundedPoint;
import frc.demacia.path.utils.Segment;
import frc.demacia.utils.Utils;

/** Add your docs here. */
public class DemaciaTrajectory {
    private ArrayList<Segment> segments;
    private double trajectoryLength;
    private double distanceTraveledOnSegment;
    private ArrayList<PathPoint> points;
    private RoundedPoint[] corners;
    private int segmentIndex;
    // private Rotation2d wantedAngle;
    public double distanceLeft;
    Pose2d chassisPose = new Pose2d();
    double accel;
    double maxVel;
    boolean isAuto;

    /*
     * 
     * given points based on blue alliance
     * 
     */
    public DemaciaTrajectory(ArrayList<PathPoint> points, boolean isRed, Pose2d initialPose) {
        this.segments = new ArrayList<Segment>();
        this.trajectoryLength = 0;
        this.distanceTraveledOnSegment = 0;
        this.points = points;
        this.segmentIndex = 0;
        this.maxVel = PathsConstraints.MAX_VELOCITY;
        this.accel = PathsConstraints.MAX_ACCEL;
        this.isAuto = DriverStation.isAutonomous();

        if (isRed)
            points = convertAlliance();
        fixFirstPoint(initialPose);
        initCorners();

        createSegments();
        trajectoryLength = calcTrajectoryLength();
        distanceLeft = trajectoryLength;

    }

    private void fixFirstPoint(Pose2d initialPose) {
        points.remove(0);
        points.add(0, new PathPoint(initialPose.getTranslation(), initialPose.getRotation(), 0));

    }

    private ArrayList<PathPoint> convertAlliance() {
        ArrayList<PathPoint> convertedPoints = new ArrayList<PathPoint>();

        for (int i = 1; i < points.size(); i++) {
            convertedPoints.add(convertPoint(points.get(i)));
        }
        return convertedPoints;

    }

    private PathPoint convertPoint(PathPoint pointToConvert) {
        return new PathPoint(
                new Translation2d(TrajectoryConstants.FIELD_LENGTH - pointToConvert.getX(),
                        TrajectoryConstants.FIELD_HEIGHT - pointToConvert.getY()),
                pointToConvert.getRotation(), pointToConvert.getWantedVelocity());
    }

    private void initCorners() {
        this.corners = new RoundedPoint[points.size() - 2];
        for (int i = 0; i < points.size() - 2; i++) {
            corners[i] = new RoundedPoint(points.get(i), points.get(i + 1), points.get(i + 2));
        }
    }

    private void createSegments() {

        if (points.size() < 3) {
            var last = points.get(points.size() - 1);
            segments.add(new Leg(points.get(0).getTranslation(),
                    last.getTranslation(),
                    last.getRotation().getRadians(),
                    last.getWantedVelocity()));
        }

        else {
            segments.add(0, corners[0].getAtoCurveLeg());

            for (int i = 0; i < corners.length - 1; i++) {
                segments.add(new Leg(corners[i].getCurveEnd(),
                        corners[i + 1].getCurveStart(),
                        corners[i + 1].getHeading1(),
                        corners[i + 1].getWantedVelocity1()));
            }

            segments.add(corners[corners.length - 1].getArc());
            segments.add(corners[corners.length - 1].getCtoCurveLeg());
        }
    }

    public double calcTrajectoryLength() {
        double sum = 0;
        for (Segment s : segments) {
            sum += s.getLength();
        }
        return sum;
    }

    public boolean hasFinishedSegments(Pose2d chassisPose, double currentVelocty) {
        Translation2d currentLastPoint = segmentIndex == segments.size() - 1
                ? segments.get(segmentIndex).getPoints()[1]
                : (segments.get(segmentIndex) instanceof Leg ? segments.get(segmentIndex).getPoints()[1]
                        : segments.get(segmentIndex + 1).getPoints()[0]);

        if (segmentIndex == segments.size() - 1)
            return chassisPose.getTranslation()
                    .getDistance(currentLastPoint) <= TrajectoryConstants.MAX_POSITION_THRESHOLD;

        else {

            return chassisPose.getTranslation().getDistance(currentLastPoint) < Utils.distanceToDeaccel(currentVelocty,
                    0, TrajectoryConstants.PathsConstraints.MAX_ACCEL);

        }
    }

    private double getVelocity(Segment currentSegment, Pose2d chassisPose, double currentVelocity) {
        double distance = currentSegment.getDistanceLeft(chassisPose.getTranslation());
        double wantedVelocity = currentSegment.getWantedVelocity();
        double vmax = Math
                .sqrt(2 * accel * distance
                        + currentVelocity * currentVelocity
                        + wantedVelocity * wantedVelocity)
                / 2;
        if (vmax > currentVelocity) {
            return MathUtil.clamp(maxVel, currentVelocity - accel * 0.02, currentVelocity + accel * 0.02);
        } else {
            double t = 2 * distance / (currentVelocity + wantedVelocity);
            frc.demacia.utils.log.LogManager.log("t: " + t + " current velocity: " + currentVelocity
                    + " wanted velocity: " + wantedVelocity + " distance: " + distance);
            if (t < 0.1) {
                return wantedVelocity;
            } else {
                return currentVelocity + (wantedVelocity - currentVelocity) * 0.1 / t;
            }
        }
    }

    double lastDistance = 0;

    public ChassisSpeeds calculate(Pose2d chassisPose, double currentVelocity) {
        this.chassisPose = chassisPose;
        var segment = segments.get(segmentIndex);

        distanceTraveledOnSegment = segment.distancePassed(chassisPose.getTranslation());
        distanceLeft -= (distanceTraveledOnSegment - lastDistance);
        lastDistance = distanceTraveledOnSegment;
        if (hasFinishedSegments(chassisPose, currentVelocity)) {
            lastDistance = 0;
            if (segmentIndex != segments.size() - 1) {
                segmentIndex++;
                segment = segments.get(segmentIndex);
            }
        }
        double velocity = getVelocity(
                segment,
                chassisPose,
                currentVelocity);

        Translation2d wantedVelocity = segments.get(segmentIndex).calcVector(chassisPose.getTranslation(), velocity);
        double diffAngle = MathUtil.angleModulus(segment.getHeading() - chassisPose.getRotation().getRadians());
        double wantedOmega = 0;

        wantedOmega = Math.abs(diffAngle) < TrajectoryConstants.MAX_ROTATION_THRESHOLD ? 0 : 1.1 * diffAngle;
        return new ChassisSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), -wantedOmega);
    }

    public boolean isFinishedTrajectory() {

        return ((chassisPose.getTranslation()
                .getDistance(
                        points.get(points.size() - 1).getTranslation()) <= TrajectoryConstants.MAX_POSITION_THRESHOLD
                && segmentIndex == segments.size() - 1));
    }

}
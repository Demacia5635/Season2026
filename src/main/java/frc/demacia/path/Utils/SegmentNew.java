// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public abstract class SegmentNew {
    Translation2d from;
    Translation2d to;
    double targteHeading;
    double wantedVelocity;
    double maxVelocity;
    double maxAccel;
    double length;
    double endAngle; // the direction of movement at the end of the segment

    public SegmentNew(Translation2d from, Translation2d to, double targteHeading, double wantedVelocity, double maxVelocity, double maxAccel) {
        this.from = from;
        this.to = to;
        this.targteHeading = targteHeading;
        this.wantedVelocity = wantedVelocity;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
    }

    record calcuateResult (double velocity, double angle, double remainingDistance) {
    }

    public abstract calcuateResult calculate(Pose2d position, ChassisSpeeds currentSpeeds);

    public double getHeading() {
        return targteHeading;
    }

    public double getWantedVelocity() {
        return this.wantedVelocity;
    }

    public double getMaxVelocity() {
        return this.maxVelocity;
    }

    public double getMaxAccel() {
        return this.maxAccel;
    }
}
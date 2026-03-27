// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;

/**
 * 
 * A point in the path
 *
 */
public class PathPoint extends Pose2dDemacia {
  public static final PathPoint kZero = new PathPoint(Translation2dDemacia.kZero, Rotation2dDemacia.kZero, 0d);

  double wantedVelocity;
  double radius;

  public PathPoint(Pose2dDemacia pose, double wantedVelocity, double radius) {
    this(pose.getX(), pose.getY(), pose.getRotation(), radius, wantedVelocity);
  }

  public PathPoint(Pose2dDemacia pose2d, double wantedVelocity) {
    this(pose2d.getTranslation(), pose2d.getRotation(), wantedVelocity);
  }

  public PathPoint(Translation2dDemacia p, Rotation2dDemacia r, double wantedVelocity) {
    this(p.getX(), p.getY(), r, 0, wantedVelocity);
  }

  public PathPoint(Translation2dDemacia p, Rotation2dDemacia r, double radius, double wantedVelocity) {
    this(p.getX(), p.getY(), r, radius, wantedVelocity);
  }

  public PathPoint(double x, double y, Rotation2dDemacia rotation, double radius, double wantedVelocity) {
    super(x, y, rotation);
    this.radius = radius;
    this.wantedVelocity = wantedVelocity;
  }

  public double getWantedVelocity() {
    return this.wantedVelocity;
  }

  public double getRadius() {
    return radius;
  }

  public void setRadius(double radius) {
    this.radius = radius;
  }

}
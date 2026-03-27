// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * 
 *A point in the path
 *
 */
public class PathPoint extends Pose2d{
    public static final PathPoint kZero = new PathPoint(Translation2d.kZero, Rotation2d.kZero, 0);

    double wantedVelocity;
    double radius;


    public PathPoint(Pose2d pose, double wantedVelocity, double radius){
      this(pose.getX(), pose.getY(), pose.getRotation(), radius, wantedVelocity);
    }
    public PathPoint(Pose2d pose2d, double wantedVelocity) {
      this(pose2d.getTranslation(), pose2d.getRotation(), wantedVelocity);
    }
    public PathPoint(Translation2d p, Rotation2d r, double wantedVelocity) {
      this(p.getX(),p.getY(),r,0, wantedVelocity);
    }
    public PathPoint(Translation2d p, Rotation2d r, double radius, double wantedVelocity) {
      this(p.getX(),p.getY(),r,radius, wantedVelocity);
    }
    public PathPoint(double x, double y, Rotation2d rotation, double radius, double wantedVelocity) {
      super(x,y,rotation);
      this.radius = radius;
      this.wantedVelocity = wantedVelocity;
    }


    public double getWantedVelocity(){
      return this.wantedVelocity;
    }

    public double getRadius()
    {
      return radius;
    }

    public void setRadius(double radius)
    {
      this.radius = radius;
    }


}
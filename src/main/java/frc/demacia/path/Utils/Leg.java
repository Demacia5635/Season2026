// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;

public class Leg extends Segment{
    Translation2dDemacia totalVector;
    final Translation2dDemacia velDirection;
    /**
     * creates a leg type segment
     * @param p1 the first point of the leg
     * @param p2 the last point of the leg
     */
    public Leg(Translation2dDemacia p1, Translation2dDemacia p2, double headingInRad, double wantedVelocity)
    {
        super(p1, p2, headingInRad, wantedVelocity);
        totalVector = p2.minus(p1);
        velDirection = totalVector.div(totalVector.getNorm());
    }

    @Override
    public Translation2dDemacia calcVector(Translation2dDemacia position, double velocity)
    {
       // if (p2.getDistance(position) <= 0.2) return new Translation2d(velocity, position.minus(p2).getAngle());
        Translation2dDemacia relativePos = position.minus(p2);
        double diffAngleMaxed = Math.min(15, p1.minus(p2).getAngle().minus(relativePos.getAngle()).getDegrees());

        return new Translation2dDemacia(velocity, relativePos.times(-1).getAngle().minus(Rotation2dDemacia.fromDegrees(diffAngleMaxed)));

    }

    @Override
    public double distancePassed(Translation2dDemacia position)
    {
        Translation2dDemacia relativePos = position.minus(p1);

        //double distanceMoved = (relativePos.getX() * velDirection.getX()) + (relativePos.getY()*velDirection.getY());
        return relativePos.getNorm();
    }

    @Override
    public double getDistanceLeft(Translation2dDemacia currentPosition){
        return p2.minus(currentPosition).getNorm();
    
    }



    @Override
    public double getLength()
    {
        return totalVector.getNorm();
    }

   

    @Override
    public String toString() {
        return "\n~Leg~\np1 : " + p1 + "\np2 : " + p2 + "\nTotalVector : " + totalVector;
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import frc.demacia.utils.geometry.Translation2dDemacia;

/** Add your docs here. */
public class Segment {
    protected Translation2dDemacia p1;
    protected Translation2dDemacia p2;
    double headingInRad;
    double wantedVelocity;

    
    public Segment(Translation2dDemacia p1, Translation2dDemacia p2, double headingInRad, double wantedVelocity)
    {
        this.p1 = p1;
        this.p2 = p2;
        this.headingInRad = headingInRad;
        this.wantedVelocity = wantedVelocity;
    }
    
    public Translation2dDemacia calcVector(Translation2dDemacia position, double velocity) {return Translation2dDemacia.kZero;}
    public double distancePassed(Translation2dDemacia position) {return 0;}
    public double getLength() {return 0;}
    public double getHeading() { return headingInRad;}
    public double getWantedVelocity(){return this.wantedVelocity;}

    public double getDistanceLeft(Translation2dDemacia currentPosition){
        return 0;
    }
    public Translation2dDemacia[] getPoints() {
        Translation2dDemacia[] pArr = {p1,p2}; 
        return pArr;
    }
    @Override
    public String toString() {
        return "\n~\np1 : " + p1 + "\np2 : " + p2;
    }
}
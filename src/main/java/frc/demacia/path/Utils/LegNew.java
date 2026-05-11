// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.Trapezoid;

/** Add your docs here. */



public class LegNew extends SegmentNew {
    public LegNew(Translation2d from, Translation2d to, double heading, double wantedVelocity, double maxVelocity, double maxAccel)
    {
        super(from, to, heading, wantedVelocity, maxVelocity, maxAccel);
        var vec = to.minus(from);
        endAngle = vec.getAngle().getRadians();
        length = vec.getNorm();
    }
    
    public calcuateResult calculate(Pose2d position, ChassisSpeeds currentSpeeds) {
        var vec = to.minus(position.getTranslation());
        double distanceLeft = vec.getNorm();
        double angleToTarget = vec.getAngle().getRadians();
        double angleError = endAngle - angleToTarget;
        if(distanceLeft < 0.1 || Math.abs(angleError) > Math.toRadians(45)) {
            return new calcuateResult(wantedVelocity, endAngle, 0);
        } else {
            double currentV = Math.hypot(currentSpeeds.vxMetersPerSecond,currentSpeeds.vyMetersPerSecond);
            double v = Trapezoid.calculate(currentV, wantedVelocity, maxVelocity, maxAccel, distanceLeft);
            double angle = 2 * angleToTarget - endAngle;
            return new calcuateResult(v, angle, distanceLeft);
        }
    }


    @Override
    public String toString() {
        return "\n~Leg~\nStartPoint : " + from + 
              "\ntoPoint : " + to +
              "\ntargetHeading : " + Math.toDegrees(targteHeading) + 
              "\nlength : " + length;
    }
}
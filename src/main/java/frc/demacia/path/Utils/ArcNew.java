// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.Trapezoid;


/** Add your docs here. */
public class ArcNew extends SegmentNew {

    Translation2d center;
    boolean isLeftTurn;
    final double radius;
  

    public ArcNew(Translation2d from, Translation2d to, Translation2d center, boolean isLeftTurn, 
            double targteHeading, double wantedVelocity, double maxVelocity, double maxAccel)
    {
      super(from, to, targteHeading, wantedVelocity, maxVelocity, maxAccel);
      this.center = center;
      this.isLeftTurn = isLeftTurn;
      Translation2d centerToStart = center.minus(from);
      Translation2d centerToEnd = center.minus(to);
      this.radius = centerToStart.getNorm();
      double fromHeading = centerToStart.getAngle().getRadians() + (isLeftTurn ? Math.PI/2 : -Math.PI/2);
      endAngle = centerToEnd.getAngle().getRadians() + (isLeftTurn ? Math.PI/2 : -Math.PI/2);
      this.length = (targteHeading - fromHeading) * radius;
        
    }


    @Override
    public String toString() {
        return "\n~Arc~\nStartPoint : " + from + 
              "\ntoPoint : " + to +
              "\ncenter : " + center + 
              "\nDirection : " + (isLeftTurn ? "Left" : "Right") +  
              "\ntargetHeading : " + Math.toDegrees(targteHeading) + 
              "\nRadius : " + radius;
    }

    public calcuateResult calculate(Pose2d position, ChassisSpeeds currentSpeeds) {
        Translation2d centerToPos = center.minus(position.getTranslation());
        double baseHeading = centerToPos.getAngle().getRadians() + (isLeftTurn ? Math.PI/2 : -Math.PI/2);
        if(Math.abs(baseHeading - targteHeading) < Math.toRadians(3)) {
          // we finished
          return new calcuateResult(wantedVelocity, targteHeading, 0  );
        }
        double r = centerToPos.getNorm();
        r = radius * 2 - r;
        double d = Math.abs(targteHeading - baseHeading) * radius;
        double currentV = Math.hypot(currentSpeeds.vxMetersPerSecond,currentSpeeds.vyMetersPerSecond);
        double v = Trapezoid.calculate(currentV, wantedVelocity, maxVelocity, maxAccel, d);
        double omega = v / r;
        double angleChange = omega * 0.03;
        double angle = baseHeading + (isLeftTurn ? angleChange : -angleChange);
        return new calcuateResult(v, angle, d);
    }

}
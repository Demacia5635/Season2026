// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.demacia.vision.Camera;

/** Add your docs here. */
public class DanielVision {

    public static Translation2d getRobotPosition(double deltaPitch, double deltaYaw, Translation3d cameraPositionOnRobot, Translation3d tagPosition, Rotation2d gyroAngle){
        Translation2d cameraToTag = cameraToTag(deltaPitch, deltaYaw, Math.abs(tagPosition.getZ() - cameraPositionOnRobot.getZ())).rotateBy(gyroAngle);
        Translation2d robotToTag = robotToTag(cameraPositionOnRobot.toTranslation2d(), cameraToTag, gyroAngle);
        Translation2d tagToRobot = robotToTag.rotateBy(Rotation2d.k180deg);
        return tagPosition.toTranslation2d().plus(tagToRobot);
    }
    private static Translation2d cameraToTag(double deltaPitch, double deltaYaw, double deltaHeight){
        double distanceFromTagX = Math.abs(deltaHeight / Math.tan(deltaPitch));
        double distanceFromTagY = distanceFromTagX * Math.tan(deltaYaw);
        return new Translation2d(distanceFromTagX, distanceFromTagY);
    }
    private static Translation2d robotToTag(Translation2d cameraPositionOnRobot, Translation2d cameraToTag, Rotation2d gyroAngle){
        Translation2d cameraOnRobotFieldRel = cameraPositionOnRobot.rotateBy(gyroAngle);
        return cameraOnRobotFieldRel.plus(cameraToTag);
    }
    


}

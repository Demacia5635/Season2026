// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.demacia.kinematics.DemaciaKinematics;
import frc.demacia.utils.chassis.Chassis;

public class ShooterUtils {

    Chassis chassis;
    DemaciaKinematics kinematics;

    public ShooterUtils(Chassis chassis, DemaciaKinematics kinematics){
        this.chassis = chassis;
        this.kinematics = kinematics;
    }

    public Translation3d fucerValRobotAsVector(ChassisSpeeds wantedVelocity){
        ChassisSpeeds currentRobotVel = chassis.getRobotRelVelocities();
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(currentRobotVel);
        ChassisSpeeds fucerRobotVel = kinematics.toChassisSpeeds(moduleStates,chassis.getGyroAngle().getDegrees());
        return new Translation3d(fucerRobotVel.vxMetersPerSecond, fucerRobotVel.vyMetersPerSecond, 0.0);
    }

    public static double distensFromToPose2dPoint(Pose2d from, Pose2d to){
        return from.minus(to).getTranslation().getNorm();
    }

    public static double angle_betuenTowPose2d(Pose2d from, Pose2d to){
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public static Pose2d getRobotFucerPose(double dtSpeed, Chassis chassis){ //TODO: finade ander solosen fot the Chassis chassis
        return chassis.computeFuturePosition(dtSpeed);
    }
}
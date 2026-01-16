// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.kinematics.DemaciaKinematics;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.geometry.Rotation2d;
import frc.demacia.utils.geometry.Translation2d;
import frc.robot.Shooter.commands.ShooterFollowCommand;
import frc.robot.Shooter.subsystem.Shooter;

public class ShooterUtils {

    Chassis chassis;
    DemaciaKinematics kinematics;

    public ShooterUtils(Chassis chassis, DemaciaKinematics kinematics){
        this.chassis = chassis;
        this.kinematics = kinematics;
    }

    public Translation3d fucerValRobot(ChassisSpeeds wantedVelocity){
        ChassisSpeeds currentRobotVel = chassis.getRobotRelVelocities();
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(currentRobotVel);
        ChassisSpeeds fucerRobotVel = kinematics.toChassisSpeeds(moduleStates,chassis.getGyroAngle().getDegrees());
        return new Translation3d(fucerRobotVel.vxMetersPerSecond, fucerRobotVel.vyMetersPerSecond, 0.0);
    }
}
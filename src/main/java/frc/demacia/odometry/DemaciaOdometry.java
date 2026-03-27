// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.odometry;

import java.util.Arrays;

import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.SwerveModulePositionDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;
import frc.demacia.utils.geometry.Twist2dDemacia;

/** Add your docs here. */
public class DemaciaOdometry {
    private Pose2dDemacia pose;
    private final Translation2dDemacia[] modulePositions;
    private SwerveModulePositionDemacia[] lastPositions;
    private Rotation2dDemacia lastAngle = Rotation2dDemacia.kZero;
    // private final AccelOdometry accelOdometry;
    private static DemaciaOdometry instance;
    private final double modulesDistanceSum;

    // private double accelParameter = 0.3;

    public DemaciaOdometry(Translation2dDemacia[] modulePositions) {
        this.modulePositions = modulePositions;
        this.lastPositions = new SwerveModulePositionDemacia[modulePositions.length];

        for (int i = 0; i < lastPositions.length; i++) {
            lastPositions[i] = new SwerveModulePositionDemacia();
        }
        // this.accelOdometry = new AccelOdometry();
        this.pose = new Pose2dDemacia();
        modulesDistanceSum = Arrays.stream(modulePositions).mapToDouble(pos -> pos.getNorm()).sum();
    }

    public static DemaciaOdometry getOdometryInstance(Translation2dDemacia[] modulePositions) {
        if (instance == null) instance = new DemaciaOdometry(modulePositions);
        return instance;
    }

    public Pose2dDemacia update(Rotation2dDemacia gyroAngle, SwerveModulePositionDemacia[] currentPositions) {
        Translation2dDemacia[] moduleDisplacements = new Translation2dDemacia[modulePositions.length];
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastPositions[i], currentPositions[i]);

        }

        Twist2dDemacia robotDisplacement = calculateRobotDisplacement(moduleDisplacements);
        robotDisplacement.dtheta = gyroAngle.minus(lastAngle).getRadians();

        pose = pose.exp(robotDisplacement);

        lastPositions = currentPositions;
        lastAngle = gyroAngle;
        return new Pose2dDemacia(pose.getTranslation(), gyroAngle);

    }

    private Translation2dDemacia calculateModuleDisplacement(SwerveModulePositionDemacia lastPosition,
            SwerveModulePositionDemacia currentPosition) {
        double arcLength = currentPosition.distanceMeters - lastPosition.distanceMeters;
        Rotation2dDemacia deltaAlpha = currentPosition.angle.minus(lastPosition.angle);
        if (Math.abs(deltaAlpha.getDegrees()) < 1E-9) {
            return new Translation2dDemacia(arcLength, currentPosition.angle);
        } else {
            Rotation2dDemacia centralAngle = deltaAlpha;
            double radius = arcLength / centralAngle.getRadians();
            double chordLength = 2 * radius * Math.sin(centralAngle.getRadians() / 2);
            Rotation2dDemacia chordAngle = lastPosition.angle.plus(centralAngle.times(0.5));

            return new Translation2dDemacia(chordLength, chordAngle);
        }
    }

    private Twist2dDemacia calculateRobotDisplacement(Translation2dDemacia[] moduleDisplacements) {
        double x = 0;
        double y = 0;
        int i = 0;
        for (Translation2dDemacia moduleDisplacement : moduleDisplacements) {
            x += moduleDisplacement.getX() * (modulePositions[i].getNorm() / modulesDistanceSum);
            y += moduleDisplacement.getY() * (modulePositions[i].getNorm() / modulesDistanceSum);
            i++;
        }
        return new Twist2dDemacia(x, y, 0);
    }

    public void resetPose() {
        resetPose(Pose2dDemacia.kZero);
    }

    public void resetPose(Pose2dDemacia pose) {
        this.pose = pose;
        lastAngle = pose.getRotation();
    }

    public Pose2dDemacia getPose2d() {
        return this.pose;
    }

}

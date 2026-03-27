// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.kinematics;

import frc.demacia.utils.geometry.ChassisSpeedsDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.SwerveModuleStateDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;

import static frc.demacia.kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class DemaciaKinematics {

    private SwerveModuleStateDemacia[] swerveStates = new SwerveModuleStateDemacia[4];
    private Pose2dDemacia startRobotPosition;
    private Translation2dDemacia[] modulePositionOnTheRobot;
    private SwerveModuleStateDemacia[] lastStates = new SwerveModuleStateDemacia[4];

    public DemaciaKinematics(Translation2dDemacia... modulePositionOnTheRobot) {
        this.startRobotPosition = Pose2dDemacia.kZero;
        this.modulePositionOnTheRobot = modulePositionOnTheRobot;
        for (int i = 0; i < 4; i++) {
            swerveStates[i] = new SwerveModuleStateDemacia();
            lastStates[i] = new SwerveModuleStateDemacia();
        }

    }

    // public SwerveModuleState[] udiTest(ChassisSpeeds wantedSpeeds, ChassisSpeeds
    // currentSpeeds) {
    // if(Math.abs(wantedSpeeds.vxMetersPerSecond) < 0.05 &&
    // Math.abs(wantedSpeeds.vyMetersPerSecond) < 0.05 &&
    // Math.abs(wantedSpeeds.omegaRadiansPerSecond)>0.01){
    // SwerveModuleState[] rotationStates = new SwerveModuleState[4];
    // for(int i = 0; i < 4; i++){
    // rotationStates[i] = new SwerveModuleState(wantedSpeeds.omegaRadiansPerSecond
    // * modulePositionOnTheRobot[i].getNorm(),
    // modulePositionOnTheRobot[i].getAngle().plus(Rotation2d.kCW_90deg));
    // }
    // return rotationStates;
    // }

    // if(KinematicsUtilities.isInRange(currentSpeeds, 0.03) &&
    // KinematicsUtilities.isInRange(wantedSpeeds, 0.03)){
    // for (SwerveModuleState swerveModuleState : lastStates) {
    // swerveModuleState.speedMetersPerSecond = 0;
    // };
    // return lastStates;
    // }
    // swerveStates = toSwerveModuleStates(wantedSpeeds);
    // lastStates = swerveStates;
    // return swerveStates;
    // }

    /**
     * transforms from chassis speeds to swerve module states (speeds in field
     * relative)
     * 
     * @param fieldRelWantedSpeeds  in field relative speeds
     * @param fieldRelCurrentSpeeds in field relative speeds
     * @param currentGyroAngle      in Rotation2d
     * 
     */
    public SwerveModuleStateDemacia[] toSwerveModuleStatesWithLimit(ChassisSpeedsDemacia fieldRelWantedSpeeds,
            ChassisSpeedsDemacia fieldRelCurrentSpeeds, Rotation2dDemacia currentGyroAngle) {

        if (isOnlyRotating(fieldRelWantedSpeeds)) {
            return onlyRotate(fieldRelWantedSpeeds);
        }
        if (isTooFastForLimit(fieldRelWantedSpeeds))
            return toSwerveModuleStates(ChassisSpeedsDemacia.fromFieldRelativeSpeeds(fieldRelCurrentSpeeds, currentGyroAngle));
        ChassisSpeedsDemacia limitedWantedVel = KinematicsUtilities.Limits.limitVelocities(fieldRelWantedSpeeds, fieldRelCurrentSpeeds);
        limitedWantedVel = ChassisSpeedsDemacia.fromFieldRelativeSpeeds(limitedWantedVel, currentGyroAngle);
        swerveStates = toSwerveModuleStates(limitedWantedVel);
        return swerveStates;
    }

    private boolean isTooFastForLimit(ChassisSpeedsDemacia wantedSpeeds) {
        return ((Math.abs(wantedSpeeds.vxMetersPerSecond) > 1.5 || Math.abs(wantedSpeeds.vyMetersPerSecond) > 1.5)
                && Math.abs(wantedSpeeds.omegaRadiansPerSecond) > Math.toRadians(40));
    }

    private boolean isOnlyRotating(ChassisSpeedsDemacia speeds) {
        return Math.abs(speeds.vxMetersPerSecond) < 0.01 && Math.abs(speeds.vyMetersPerSecond) < 0.01
                && Math.abs(speeds.omegaRadiansPerSecond) > 0.01;
    }

    private SwerveModuleStateDemacia[] onlyRotate(ChassisSpeedsDemacia speeds) {
        SwerveModuleStateDemacia[] rotationStates = new SwerveModuleStateDemacia[4];
        for (int i = 0; i < 4; i++) {
            rotationStates[i] = new SwerveModuleStateDemacia(
                    speeds.omegaRadiansPerSecond * modulePositionOnTheRobot[i].getNorm(),
                    modulePositionOnTheRobot[i].getAngle().plus(Rotation2dDemacia.kCW_90deg));
        }
        return rotationStates;
    }

    public ChassisSpeedsDemacia toChassisSpeeds(SwerveModuleStateDemacia[] swerveStates, double omegaFromGyro) {
        double sumVx = 0;
        double sumVy = 0;

        for (int i = 0; i < 4; i++) {
            double angleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double distanceFromCenter = modulePositionOnTheRobot[i].getNorm();
            double currentAngle = swerveStates[i].angle.getRadians();
            double moduleVx = swerveStates[i].speedMetersPerSecond * Math.cos(currentAngle);
            double moduleVy = swerveStates[i].speedMetersPerSecond * Math.sin(currentAngle);

            double chassisVx = moduleVx - (omegaFromGyro * distanceFromCenter
                    * Math.sin(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));
            double chassisVy = moduleVy + (omegaFromGyro * distanceFromCenter
                    * Math.cos(currentAngle + (omegaFromGyro * 0.02) + angleFromCenter));

            sumVx += chassisVx;
            sumVy += chassisVy;
        }
        return new ChassisSpeedsDemacia(sumVx / 4.0, sumVy / 4.0, omegaFromGyro);
    }

    public SwerveModuleStateDemacia[] toSwerveModuleStates(ChassisSpeedsDemacia wantedSpeeds) {

        double omega = wantedSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            double moduleAngleFromCenter = modulePositionOnTheRobot[i].getAngle().getRadians();
            double moduleCurrentAngle = startRobotPosition.getRotation().getRadians();
            Translation2dDemacia velocityVector = new Translation2dDemacia(
                    wantedSpeeds.vxMetersPerSecond + omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.sin(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter),
                    wantedSpeeds.vyMetersPerSecond - omega * modulePositionOnTheRobot[i].getNorm()
                            * Math.cos(moduleCurrentAngle + omega * 0.02 + moduleAngleFromCenter));
            swerveStates[i] = new SwerveModuleStateDemacia(velocityVector.getNorm(), new Rotation2dDemacia(
                    KinematicsUtilities.getAngleFromVector(velocityVector.getX(), velocityVector.getY())));
        }

        swerveStates = factorModuleVelocities(swerveStates);
        return swerveStates;
    }

    private SwerveModuleStateDemacia[] factorModuleVelocities(SwerveModuleStateDemacia[] swerveStates) {
        double maxVelocityCalculated = 0;
        for (int i = 0; i < swerveStates.length; i++) {
            double cur = Math.abs(swerveStates[i].speedMetersPerSecond);
            if (cur == 0)
                return swerveStates;
            if (cur > maxVelocityCalculated)
                maxVelocityCalculated = cur;
        }
        double factor = MAX_ALLOWED_MODULE_VELOCITY / maxVelocityCalculated;

        if (factor >= 1)
            return swerveStates;

        for (SwerveModuleStateDemacia state : swerveStates) {
            state.speedMetersPerSecond = state.speedMetersPerSecond * factor;
        }
        return swerveStates;

    }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.odometry;

import static frc.demacia.vision.utils.VisionConstants.BEST_RELIABLE_SPEED;
import static frc.demacia.vision.utils.VisionConstants.WORST_RELIABLE_SPEED;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.odometry.DemaciaPoseEstimator.OdometryObservation;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.subsystem.Quest;
import frc.demacia.vision.utils.Vision;
import frc.demacia.vision.utils.VisionConstants;
import frc.demacia.vision.utils.LimelightHelpers.PoseEstimate;
import frc.robot.Field;
import frc.robot.RobotCommon;
import frc.robot.Turret.Turret;

/** Add your docs here. */
public class RobotPose {
    private static RobotPose instance;

    private Vision vision;
    private DemaciaPoseEstimator poseEstimator;
    private Quest quest;

    private Matrix<N3, N1> questSTD;

    private boolean hasUpdatedQuestIntialPose;
    private boolean hasQuestDisconnected;

    private Matrix<N3, N1> visionSTD;
    private BuiltInAccelerometer accelerometer;

    private RobotPose(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {
        this.vision = new Vision((VisionConstants.Tags.TAGS_ARRAY));

        this.quest = new Quest();
        this.questSTD = questSTD;
        this.visionSTD = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 0.3, 0.3, 999999 }));
        this.hasUpdatedQuestIntialPose = false;
        this.hasQuestDisconnected = false;
        this.poseEstimator = new DemaciaPoseEstimator(modulePositions, stateSTD, visionSTD);
        this.accelerometer = new BuiltInAccelerometer();
        SmartDashboard.putData("Reset Pose Based Red Hub", new InstantCommand(() -> {
            Chassis.getInstance().setYaw(Rotation2d.kZero);
            setQuestPose(hubRedResetPose);
            resetPose(hubRedResetPose);
        }).ignoringDisable(true));
    }
    private final Pose2d hubRedResetPose = new Pose2d(Field.HubRed.X_BACK + 0.3, Field.HubRed.Y_CENTER, Rotation2d.kZero);

    public Quest getQuest() {
        return quest;
    }

    public Pose2d getPose() {
        // if(hasUpdatedQuestIntialPose && quest.isConnected()) return quest.getRobotPose2d();
        
        return poseEstimator.getEstimatedPose();
    }

    public static void initialize(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {

        if (instance == null)
            instance = new RobotPose(modulePositions, stateSTD, questSTD);
    }

    public void resetPose() {
        resetPose(Pose2d.kZero);
    }

    public void resetPose(Pose2d pose) {
        System.out.println(pose);
        poseEstimator.resetPose(pose);
    }

    public static RobotPose getInstance() {
        return instance;
    }

    public void addOdometryCalculation(OdometryObservation odometryObservation) {
        poseEstimator.addOdometryCalculation(odometryObservation);
    }

    public void addOdometryCalculation(Pose2d odometryPose, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {
        addOdometryCalculation(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions));
    }

    public void setQuestPose() {
        // if (vision.isSeeTag()) {
        //     setQuestPose(vision.getPoseEstimation());
        // }
        // else{
            setQuestPose(getPose());
        // }
    }

    public void setQuestPose(Pose2d pose) {
        hasUpdatedQuestIntialPose = true;
        quest.setQuestPose(new Pose3d(pose));
    }

    public void addVisionMeasurement() {

        // if (!hasUpdatedQuestIntialPose && visionCounter > 30) {
        // hasUpdatedQuestIntialPose = true;
        // setQuestPose();
        // }

        poseEstimator.setVisionMeasurementStdDevs(visionSTD);
        poseEstimator.addVisionMeasurement(vision.getPoseEstimation(), Timer.getFPGATimestamp() - 0.05);
    }

    public void addQuestMeasurement() {
        poseEstimator.setVisionMeasurementStdDevs(questSTD);
        poseEstimator.addVisionMeasurement(quest.getRobotPose2d(), Timer.getFPGATimestamp() - 0.05);

    }

    public void update(Pose2d odometryPose, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, Translation2d currentVelocity) {
        update(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions));

    }

    private boolean shouldUpdateVision() {
        // return (Math.hypot(RobotCommon.fieldRelativeSpeeds.vxMetersPerSecond,
        // RobotCommon.fieldRelativeSpeeds.vyMetersPerSecond) <= 3
        // // && Turret.getInstance().getTurretVelocity() <= Math.toRadians(100)
        // && vision.isSeeTagWithDistance());

        return vision.isSeeTag();// && Turret.getInstance().hasCalibrated();

    }

    public void update(OdometryObservation odometryObservation) {

        if (Math.abs(accelerometer.getX()) < 1.8 && Math.abs(accelerometer.getZ()) < 1.8)
            addOdometryCalculation(odometryObservation);

        vision.updateValues();

        if (hasUpdatedQuestIntialPose && quest.isConnected()) {

            addQuestMeasurement();
        }
        if (shouldUpdateVision()) {

            addVisionMeasurement();
            if (hasQuestDisconnected && quest.isConnected()) {
                setQuestPose();
                hasQuestDisconnected = false;
            }
        }
        if (!hasQuestDisconnected && !quest.isConnected()) {
            hasQuestDisconnected = true;
        }
    }

    private static Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;

        ChassisSpeeds currentSpeeds = RobotCommon.robotRelativeSpeeds;
        double speed = Utilities.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // Vision confidence adjustment
        // if (visionFuse != null && visionFuse.getVisionConfidence() < 0.3) {
        // x += 0.3;
        // y += 0.3;
        // }

        // Speed-based confidence calculation
        if (speed > WORST_RELIABLE_SPEED) {
            // Maximum uncertainty for high speeds
            x += 0.02;
            y += 0.02;
        } else if (speed <= BEST_RELIABLE_SPEED) {
            // Minimum uncertainty for low speeds
            x -= 0.02;
            y -= 0.02;
        } else {
            // Calculate normalized speed for the falloff range
            double normalizedSpeed = (speed - BEST_RELIABLE_SPEED)
                    / (WORST_RELIABLE_SPEED - BEST_RELIABLE_SPEED);

            // Apply exponential falloff to calculate additional uncertainty
            double speedConfidence = Math.exp(-3 * normalizedSpeed);

            // Scale the uncertainty adjustment based on confidence
            double adjustment = 0.02 * (1 - speedConfidence);
            x += adjustment;
            y += adjustment;
        }

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { x, y, theta }));
    }

}

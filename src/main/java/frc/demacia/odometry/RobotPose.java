// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.odometry;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.demacia.odometry.DemaciaPoseEstimator.OdometryObservation;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Pose3dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.SwerveModulePositionDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;
import frc.demacia.vision.subsystem.Quest;
import frc.demacia.vision.utils.Vision;
import frc.demacia.vision.utils.VisionConstants;

import frc.robot.RobotContainer;

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

    private RobotPose(Translation2dDemacia[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {
        this.vision = new Vision((VisionConstants.Tags.TAGS_ARRAY));

        this.quest = new Quest();
        this.questSTD = questSTD;
        this.visionSTD = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 0.3, 0.3, 0 }));
        this.hasUpdatedQuestIntialPose = false;
        this.hasQuestDisconnected = false;
        this.poseEstimator = new DemaciaPoseEstimator(modulePositions, stateSTD, visionSTD);
        this.accelerometer = new BuiltInAccelerometer();
        
        SmartDashboard.putData("full reset pose", fullResetPoseCommand());
    }

    public Command fullResetPoseCommand() {
        return Commands.sequence(
            new InstantCommand(() -> vision.setToDimension(true)),
            new WaitCommand(1),
            new InstantCommand(this::setAngle3DLimelight),
            new WaitCommand(1),
            new InstantCommand(() -> vision.setToDimension(false)),
            new WaitCommand(1),
            new InstantCommand(this::setQuestPose)
        ).ignoringDisable(true);
    }

    public Vision getVision() {
        return vision;
    }

    public Quest getQuest() {
        return quest;
    }

    public Pose2dDemacia getPose() {

        return poseEstimator.getEstimatedPose();
    }

    public static void initialize(Translation2dDemacia[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {

        if (instance == null)
            instance = new RobotPose(modulePositions, stateSTD, questSTD);
    }

    public void resetPose() {
        resetPose(Pose2dDemacia.kZero);
    }

    public void resetPose(Pose2dDemacia pose) {
        poseEstimator.resetPose(pose);
    }

    public static RobotPose getInstance() {
        return instance;
    }

    public void addOdometryCalculation(OdometryObservation odometryObservation) {
        poseEstimator.addOdometryCalculation(odometryObservation);
    }

    public void addOdometryCalculation(Pose2dDemacia odometryPose, Rotation2dDemacia gyroAngle,
            SwerveModulePositionDemacia[] modulePositions) {
        addOdometryCalculation(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions));
    }

    public void setQuestPose() {
        if (vision.isSeeTag()) {
            setQuestPose(vision.getPoseEstimation());
        } else {
            setQuestPose(getPose());
        }
    }

    public void setQuestHeading(Rotation2dDemacia heading) {
        quest.setHeading(heading);
    }

    public void setQuestPose(Pose2dDemacia pose) {
        hasUpdatedQuestIntialPose = true;
        quest.setQuestPose(new Pose3dDemacia(pose));
    }

    public void addVisionMeasurement(Rotation2dDemacia gyroAngle) {
        poseEstimator.setVisionMeasurementStdDevs(visionSTD);
        poseEstimator.addVisionMeasurement(
                new Pose2dDemacia(vision.getPoseEstimation().getX(), vision.getPoseEstimation().getY(), gyroAngle),
                Timer.getFPGATimestamp() - 0.05);
    }

    public void addQuestMeasurement(Rotation2dDemacia gyroAngle) {
        poseEstimator.setVisionMeasurementStdDevs(questSTD);
        poseEstimator.addVisionMeasurement(
                new Pose2dDemacia(quest.getRobotPose2d().getX(), quest.getRobotPose2d().getY(), gyroAngle),
                Timer.getFPGATimestamp() - 0.05);

    }

    public void update(Pose2dDemacia odometryPose, Rotation2dDemacia gyroAngle,
            SwerveModulePositionDemacia[] modulePositions, Translation2dDemacia currentVelocity) {
        update(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions));

    }

    private boolean shouldUpdateVision() {
        return vision.isSeeTag();

    }

    public void setAngle3DLimelight() {
        Rotation2dDemacia newAngle = vision.getRobotAngle();
        if (newAngle != null)
            Chassis.getInstance().setYaw(newAngle);

    }

    public void update(OdometryObservation odometryObservation) {

        vision.updateValues();
        if (!quest.isConnected())
            RobotContainer.getMainLeds().isQuestDisconnected = true;

        if (Math.abs(accelerometer.getX()) < 1.8 && Math.abs(accelerometer.getZ()) < 1.8)
            addOdometryCalculation(odometryObservation);

        if (hasUpdatedQuestIntialPose && quest.isConnected()) {

            addQuestMeasurement(odometryObservation.gyroAngle());
        }
        if (shouldUpdateVision()) {

            addVisionMeasurement(odometryObservation.gyroAngle());
            if (hasQuestDisconnected && quest.isConnected()) {
                // setQuestPose();
                hasQuestDisconnected = false;
            }
        }
        if (!hasQuestDisconnected && !quest.isConnected()) {
            hasQuestDisconnected = true;
        }
    }
}

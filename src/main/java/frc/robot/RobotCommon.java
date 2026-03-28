package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.geometry.ChassisSpeedsDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;

public class RobotCommon {
    public enum Shifts {
        Auto, Transition, Active, Inactive, Endgame, Disable;
    }

    public enum RobotStates {
        Idle, Hub, Delivery, DriveWithIntake,
        Trench, Test;
    }

    private static RobotStates state = RobotStates.Idle;
    private static Pose2dDemacia currentRobotPose = Pose2dDemacia.kZero;
    private static Pose2dDemacia futureRobotPose = Pose2dDemacia.kZero; // 0.04 seconds in advance
    private static ChassisSpeedsDemacia fieldRelativeSpeeds = new ChassisSpeedsDemacia();
    private static Rotation2dDemacia robotAngle = Rotation2dDemacia.kZero;
    private static double currentDistanceFromTarget = 0;
    private static double futureAngleFromTargetRobotRelative = 0;
    private static boolean isStuck = false;

    private static boolean isRed = true;
    private static boolean isComp = false;
    private static Shifts shift = Shifts.Auto;
    private static Shifts nextShift = Shifts.Transition;

    private static boolean hasReady = false;

    public static boolean isReady() {
        // return true;
        return (StateManager.getInstance().getTimeLeft() <= 3 || !shift.equals(Shifts.Inactive)
                || !state.equals(RobotStates.Hub)) && ((Turret.getInstance().isReady()
                && Shooter.getInstance().isReady()));
    }

    public static void changeShift(Shifts newShift, Shifts nextShift) {
        RobotContainer.getMainLeds().isShiftEnded = true;
        shift = newShift;
        RobotCommon.nextShift = nextShift;
    }

    public static Command changeStateCommand(RobotStates newState) {
        return new InstantCommand(() -> {
            StateManager.getInstance().setStateChangeActivated(false);
            setState(newState);
        }).ignoringDisable(true);
    }

    public static boolean isReadyToShoot() {
        return Turret.getInstance().isReady() && Shooter.getInstance().isReady();
    }

    public static boolean isRobotFunctional() {
        return Turret.getInstance().hasCalibrated() && RobotPose.getInstance().getQuest().isWorking()
                && RobotCommon.currentRobotPose != Pose2dDemacia.kZero;
    }

    public static RobotStates getState() {
        return state;
    }

    public static void setState(RobotStates currentState) {
        RobotCommon.state = currentState;
    }

    public static Pose2dDemacia getCurrentRobotPose() {
        return currentRobotPose;
    }

    public static void setCurrentRobotPose(Pose2dDemacia currentRobotPose) {
        RobotCommon.currentRobotPose = currentRobotPose;
    }

    public static Pose2dDemacia getFutureRobotPose() {
        return futureRobotPose;
    }

    public static void setFutureRobotPose(Pose2dDemacia futureRobotPose) {
        RobotCommon.futureRobotPose = futureRobotPose;
    }

    public static ChassisSpeedsDemacia getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    public static void setFieldRelativeSpeeds(ChassisSpeedsDemacia fieldRelativeSpeeds) {
        RobotCommon.fieldRelativeSpeeds = fieldRelativeSpeeds;
    }

    public static Rotation2dDemacia getRobotAngle() {
        return robotAngle;
    }

    public static void setRobotAngle(Rotation2dDemacia robotAngle) {
        RobotCommon.robotAngle = robotAngle;
    }

    public static double getCurrentDistanceFromTarget() {
        return currentDistanceFromTarget;
    }

    public static void setCurrentDistanceFromTarget(double currentDistanceFromTarget) {
        RobotCommon.currentDistanceFromTarget = currentDistanceFromTarget;
    }

    public static double getFutureAngleFromTargetRobotRelative() {
        return futureAngleFromTargetRobotRelative;
    }

    public static void setFutureAngleFromTargetRobotRelative(double futureAngleFromTargetRobotRelative) {
        RobotCommon.futureAngleFromTargetRobotRelative = futureAngleFromTargetRobotRelative;
    }

    public static boolean isStuck() {
        return isStuck;
    }

    public static void setStuck(boolean isStuck) {
        RobotCommon.isStuck = isStuck;
    }

    public static boolean isRed() {
        return isRed;
    }

    public static void setRed(boolean isRed) {
        RobotCommon.isRed = isRed;
    }

    public static boolean isComp() {
        return isComp;
    }

    public static void setComp(boolean isComp) {
        RobotCommon.isComp = isComp;
    }

    public static Shifts getShift() {
        return shift;
    }

    public static void setShift(Shifts currentShift) {
        RobotCommon.shift = currentShift;
    }

    public static Shifts getNextShift() {
        return nextShift;
    }

    public static void setNextShift(Shifts nextShift) {
        RobotCommon.nextShift = nextShift;
    }

    public static boolean isHasReady() {
        return hasReady;
    }

    public static void setHasReady(boolean hasReady) {
        RobotCommon.hasReady = hasReady;
    }
}
package frc.robot.auto;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.demacia.path.Trgectory.FollowTrajectory;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Field;
import frc.robot.RobotCommon;
import frc.robot.StateManager;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

public class DuchToBalls extends SequentialCommandGroup {
    private final Chassis chassis;
    private final IntakeSubsystem intake;
    private final ShinuaSubsystem shinua;
    private final Turret turret;
    private final Shooter shooter;

    private final PathPoint trenchHelper = new PathPoint(new Pose2d(8.716, Field.TrenchRedScoring.Y_CENTER, Rotation2d.fromDegrees(125)));
    private final PathPoint[] traj1 = {
            PathPoint.kZero,
            new PathPoint(
                    new Pose2d(10.6, Field.TrenchRedScoring.Y_CENTER, Rotation2d.kCCW_90deg)),
            // trenchHelper,
            // new PathPoint(new Pose2d(Field.Zones.CENTER_LINE_X, Field.BumpRedScoring.Y_CENTER, Rotation2d.fromDegrees(125)))
    };

    public DuchToBalls(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinua,
            Turret turret, Shooter shooter) {
        this.chassis = chassis;
        this.intake = intake;
        this.shinua = shinua;
        this.turret = turret;
        this.shooter = shooter;
        StateManager.getInstance().setStateChangeActivated(false);
        // addCommands(new FunctionalCommand(
        //         () -> {
        //             RobotCommon.currentState = RobotStates.HubWithoutAutoIntake;
        //         },
        //         () -> {
        //         }, (interuppted) -> {
        //             RobotCommon.currentState = RobotStates.Trench;
        //         }, () -> false).withTimeout(2));

        addCommands(new FollowTrajectory(chassis, new ArrayList<>(Arrays.asList(traj1)), Rotation2d.fromDegrees(-75)));
    }
}

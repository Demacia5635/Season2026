package frc.robot.Turret.TurretCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotCommon;
import frc.robot.Turret.Turret;

public class TurretCommand extends Command {

    private final Turret turret;
    private double wantedAngle;

    public TurretCommand(Turret turret) {
        this.turret = turret;
        this.wantedAngle = 0;

        addRequirements(turret);
        SmartDashboard.putData("Turret Command", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Wanted Turret Angle", () -> wantedAngle, (x) -> wantedAngle = x);
    }

    @Override
    public void initialize() {
        wantedAngle = Math.toDegrees(turret.getTurretAngle());
    }

    @Override
    public void execute() {
        switch (RobotCommon.getState()) {
            case Hub, Delivery:
                // turret.setPositionMotion(RobotCommon.getFutureAngleFromTargetRobotRelative());
                turret.setPositionPID(RobotCommon.getFutureAngleFromTargetRobotRelative());
                 break;

            case Test:
                turret.setPositionMotion((Math.toRadians(wantedAngle)));
                break;

            default:
                turret.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

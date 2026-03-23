package frc.robot.Turret.TurretCommands;

import java.util.logging.LogManager;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.vision.TagPose;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretConstants;

public class TurretCommand extends Command {
    private final Turret turret;
    private double wantedAngle = 0;
    private Timer timer;

    public TurretCommand(Turret turret) {
        this.turret = turret;
        this.timer = new Timer();
        addRequirements(turret);
        SmartDashboard.putData("Turret Command", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Wanted Turret Angle", () -> wantedAngle, (x) -> wantedAngle = x);
    }

    @Override
    public void initialize() {
        timer.reset();
        wantedAngle = Math.toDegrees(turret.getTurretAngle());
    }

    private boolean isTurretAtLimit() {
        return turret.isAtMaxLimit() || turret.isAtMinLimit();
    }

    private void handleTurretAtLimit() {
        if (turret.isAtMaxLimit())
            handleTurretAtLimit(true);
        else
            handleTurretAtLimit(false);
    }

    private void handleTurretAtLimit(boolean isAtMaxLimit) {

        if (!timer.isRunning()) {

            turret.updatePositionByLimit();
            timer.start();

        }
        if (timer.hasElapsed(0.1))
            turret.setPower(isAtMaxLimit ? -0.2 : 0.2);
        else
            turret.stop();
    }

    @Override
    public void execute() {

        // if (isTurretAtLimit())
        // handleTurretAtLimit(turret.isAtMaxLimit());
        // else {

        switch (RobotCommon.currentState) {
            case Hub, Delivery:
                turret.setPositionPID(RobotCommon.futureAngleFromTargetRobotRelative);
                // turret.setPositionFieldRelative(RobotCommon.futureAngleFromTarget);
                break;

            case Drive:
                turret.setPositionPID(Math.PI);
                break;
            case Test:
                turret.setPositionPID((Math.toRadians(wantedAngle)));
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

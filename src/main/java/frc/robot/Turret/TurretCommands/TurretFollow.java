package frc.robot.Turret.TurretCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.geometry.Translation2dDemacia;
import frc.robot.RobotCommon;
import frc.robot.Turret.Turret;

public class TurretFollow extends Command {

    private final Turret turret;
    private final Translation2dDemacia target;

    public TurretFollow(Turret turret, Translation2dDemacia target) {
        this.turret = turret;
        this.target = target;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setPositionMotion(MathUtil
                .angleModulus(target.minus(RobotCommon.getFutureRobotPose().getTranslation()).getAngle().getRadians())
                - RobotCommon.getFutureRobotPose().getRotation().getRadians());

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

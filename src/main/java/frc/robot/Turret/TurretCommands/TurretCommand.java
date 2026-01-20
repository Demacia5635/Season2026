package frc.robot.Turret.TurretCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Turret.Turret;

public class TurretCommand extends Command{
    Turret turret;
    double wantedAngle = 0;
    public TurretCommand(){
        this.turret = Turret.getInstance();
        addRequirements(turret);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Wanted Turret Angle", ()->wantedAngle, (x)->wantedAngle = x);
    }

    @Override
    public void execute() {
        if(!turret.hasCalibrated()) return;
        turret.setPositionMotion(wantedAngle);
    }


}

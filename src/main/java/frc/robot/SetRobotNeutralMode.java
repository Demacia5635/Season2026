package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

public class SetRobotNeutralMode extends Command {
    private final Chassis chassis;
    private final IntakeSubsystem intake;
    private final ShinuaSubsystem shinua;
    private final Turret turret;
    private final Shooter shooter;

    private boolean isBrake;

    public SetRobotNeutralMode(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinua, Turret turret, Shooter shooter) {
        this.chassis = chassis;
        this.intake = intake;
        this.shinua = shinua;
        this.turret = turret;
        this.shooter = shooter;

        isBrake = true;
    }

    @Override
    public void initialize() {
        chassis.setNeutralMode(isBrake);
        intake.setNeutralMode(isBrake);
        shinua.setNeutralMode(isBrake);
        turret.setNeutralMode(isBrake);
        shooter.setNeutralMode(isBrake);
        
        isBrake = !isBrake;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

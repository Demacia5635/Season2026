package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.RobotBLedStrip;

public class SetRobotNeutralMode extends Command {
    private final Chassis chassis;
    private final IntakeSubsystem intake;
    private final ShinuaSubsystem shinua;
    private final Turret turret;
    private final Shooter shooter;

    private final RobotBLedStrip leds;

    private boolean isBrake;

    public SetRobotNeutralMode(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinua, Turret turret, Shooter shooter, RobotBLedStrip leds) {
        this.chassis = chassis;
        this.intake = intake;
        this.shinua = shinua;
        this.turret = turret;
        this.shooter = shooter;
        this.leds = leds;

        isBrake = true;
    }

    @Override
    public void initialize() {
        isBrake = !isBrake;

        leds.startUserButton();

        chassis.setNeutralMode(isBrake);
        intake.setNeutralMode(isBrake);
        shinua.setNeutralMode(isBrake);
        turret.setNeutralMode(isBrake);
        shooter.setNeutralMode(isBrake);

        
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

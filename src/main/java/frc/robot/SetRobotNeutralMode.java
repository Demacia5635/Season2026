package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;

public class SetRobotNeutralMode extends Command {
    private final Chassis chassis;

    private boolean isBrake;

    public SetRobotNeutralMode(Chassis chassis) {
        this.chassis = chassis;
    }
    
    @Override
    public void initialize() {
        isBrake = !isBrake;
        chassis.setNeutralMode(isBrake);
        
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

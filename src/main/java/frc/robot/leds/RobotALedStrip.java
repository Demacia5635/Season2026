package frc.robot.leds;

import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotContainer;

public class RobotALedStrip extends LedStrip {
    
    public RobotALedStrip() {
        super("Robot A Strip", 14, RobotContainer.ledManager);
    }

    @Override
    public void periodic() {
        super.periodic();
        setSolidGay();
    }
}

package frc.robot.leds;

import frc.demacia.utils.leds.LedManager;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotContainer;

public class DianasourLedStrip extends LedStrip {
    
    public DianasourLedStrip() {
        super("Dianasour", 17, RobotContainer.ledManager, 24);
    }

    @Override
    public void periodic() {
        super.periodic();
        setGay();
    }
}

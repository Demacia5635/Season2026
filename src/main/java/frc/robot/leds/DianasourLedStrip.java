package frc.robot.leds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon;
import frc.robot.RobotContainer;

public class DianasourLedStrip extends LedStrip {

    public DianasourLedStrip() {
        super("Dianasour", 17, RobotContainer.getLedManager(), 24);
        
        SmartDashboard.putData("DianasourLedStrip", this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    private Color getColorBasedOnState(RobotCommon.RobotStates state) {
        return switch (state) {
            case Trench -> Color.kGreen;
            case Hub -> Color.kBlue;
            case Delivery -> Color.kRed;
            case DriveWithIntake -> Color.kYellow;
            default -> Color.kPurple;
        };
    }

    public void changeColor(RobotCommon.RobotStates state) {
        setColor(getColorBasedOnState(state));
    }
}

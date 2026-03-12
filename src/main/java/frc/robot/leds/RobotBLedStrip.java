package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon;
import frc.robot.RobotContainer;

public class RobotBLedStrip extends LedStrip {

    public RobotBLedStrip() {
        super("Robot B Strip", 14, RobotContainer.ledManager);
    }

    private Color getColorBasedOnState(RobotCommon.RobotStates state) {
        switch (RobotCommon.currentState) {
            case Drive:
                return Color.kWhiteSmoke;
            case Idle:
                return Color.kPurple;
            case Test:
                return Color.kDeepPink;
            case Trench:
                return Color.kGreen;
            default:
                return Color.kBlack;
        }
    }

    public void changeColor(RobotCommon.RobotStates state) {
        setSolidGay();
        // CommandScheduler.getInstance()
        //         .schedule(new RunCommand(() -> setBlink(getColorBasedOnState(state)), this).ignoringDisable(true)
        //                 .withTimeout(2).andThen(new RunCommand(() -> setColor(getColorBasedOnState(state)), this))
        //                 .ignoringDisable(true));
    }
}

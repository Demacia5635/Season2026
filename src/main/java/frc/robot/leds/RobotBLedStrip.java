package frc.robot.leds;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.leds.LedConstants;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon;
import frc.robot.RobotContainer;
import frc.robot.StateManager;
import frc.robot.RobotCommon.Shifts;

public class RobotBLedStrip extends LedStrip {

    public RobotBLedStrip() {
        super("Robot B Strip", LedConstants.LENGTH, RobotContainer.ledManager);
        this.transitionTimer = new Timer();
        this.ourShiftTimer = new Timer();
        this.theirShiftTimer = new Timer();
        this.endGameTimer = new Timer();
        this.disableTimer = new Timer();

        SmartDashboard.putData("Main Leds", this);
    }

    Timer transitionTimer;
    Timer ourShiftTimer;
    Timer theirShiftTimer;
    Timer endGameTimer;
    Timer disableTimer;

    public void startTransition() {
        transitionTimer.reset();
        transitionTimer.start();
    }

    public void startOurShift() {
        ourShiftTimer.reset();
        ourShiftTimer.start();
    }

    public void startTheirShift() {
        theirShiftTimer.reset();
        theirShiftTimer.start();
    }

    public void startEndGame() {
        endGameTimer.reset();
        endGameTimer.start();
    }

    public void startDisable() {
        disableTimer.reset();
        disableTimer.start();
    }

    public void startShift(Shifts shift) {
        switch (shift) {
            case Transition:
                startTransition();
                break;
            
            case Active:
                startOurShift();
                break;

            case Inactive:
                startTheirShift();
                break;
            
            case Endgame:
                startEndGame();
                break;

            case Disable:
                startDisable();
                break;

            default:
                break;
        }
    }

    private final double TIME_TO_BLINK = 3;

    @Override
    public void periodic() {
        super.periodic();

        setColor(Color.kGreen);

        if (StateManager.getInstance().getTimeLeft() <= TIME_TO_BLINK) {
            startShift(RobotCommon.nextShift);
        }

        if (!RobotCommon.isReady()) {
            setGay();
            // setColor(Color.kRed);
        } 

        if (transitionTimer.isRunning()) {
            setBlink(Color.kPurple);
        }

        if (transitionTimer.hasElapsed(TIME_TO_BLINK)) {
            transitionTimer.stop();
            transitionTimer.reset();
        }

        if (ourShiftTimer.isRunning()) {
            setBlink(Color.kDarkRed);
        }

        if (ourShiftTimer.hasElapsed(TIME_TO_BLINK)){
            ourShiftTimer.stop();
            ourShiftTimer.reset();
        }

        if (theirShiftTimer.isRunning()) {
            setBlink(Color.kDarkBlue);
        }
        
        if (theirShiftTimer.hasElapsed(TIME_TO_BLINK)) {
            theirShiftTimer.stop();
            theirShiftTimer.reset();
        }

        if (endGameTimer.isRunning()) {
            setBlink(Color.kYellow);
        }

        if (endGameTimer.hasElapsed(TIME_TO_BLINK)) {
            endGameTimer.stop();
            endGameTimer.reset();
        }

        if (disableTimer.isRunning()) {
            setBlink(Color.kWhite);
        }
        
        if (RobotState.isDisabled()) {
            disableTimer.stop();
            disableTimer.reset();
        }
    }
}
    
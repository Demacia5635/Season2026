package frc.robot.leds;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.demacia.utils.leds.LedConstants;
import frc.demacia.utils.leds.LedStrip;

import frc.robot.RobotCommon;
import frc.robot.RobotCommon.Shifts;
import frc.robot.RobotContainer;
import frc.robot.StateManager;

public class RobotBLedStrip extends LedStrip {

    public RobotBLedStrip() {
        super("Robot B Strip", LedConstants.LENGTH, RobotContainer.getLedManager());
        this.transitionTimer = new Timer();
        this.ourShiftTimer = new Timer();
        this.theirShiftTimer = new Timer();
        this.endGameTimer = new Timer();
        this.disableTimer = new Timer();

        this.userButtonTimer = new Timer();
        this.shiftEndedTimer = new Timer();

        SmartDashboard.putData("Main Leds", this);
        SmartDashboard.putData("Quest Connected", new InstantCommand(() -> isQuestDisconnected = false).ignoringDisable(true));
    }

    Timer transitionTimer;
    Timer ourShiftTimer;
    Timer theirShiftTimer;
    Timer endGameTimer;
    Timer disableTimer;
    Timer shiftEndedTimer;

    Timer userButtonTimer;

    public void startUserButton() {
        userButtonTimer.start();
    }

    private void startTransition() {
        transitionTimer.reset();
        transitionTimer.start();
    }

    private void startOurShift() {
        ourShiftTimer.reset();
        ourShiftTimer.start();
    }

    private void startTheirShift() {
        theirShiftTimer.reset();
        theirShiftTimer.start();
    }

    private void startEndGame() {
        endGameTimer.reset();
        endGameTimer.start();
    }

    private void startDisable() {
        disableTimer.reset();
        disableTimer.start();
    }

    public void startShift(Shifts shift) {
        switch (shift) {
            case Transition:
                if (!transitionTimer.isRunning())
                    startTransition();
                break;

            case Active:
                if (!ourShiftTimer.isRunning())
                    startOurShift();
                break;

            case Inactive:
                if (!theirShiftTimer.isRunning())
                    startTheirShift();
                break;

            case Endgame:
                if (!endGameTimer.isRunning())
                    startEndGame();
                break;

            case Disable:
                if (!disableTimer.isRunning())
                    startDisable();
                break;

            default:
                break;
        }
    }

    public boolean isQuestDisconnected = false;

    public boolean isShiftEnded = false;

    private final double TIME_TO_BLINK = 10;

    @Override
    public void periodic() {
        super.periodic();

        setColor(Color.kPurple);

        if (StateManager.getInstance().getTimeLeft() <= TIME_TO_BLINK) {
            startShift(RobotCommon.getNextShift());
        }

        if (!RobotCommon.isReady()) {
            // setGay();
            setColor(Color.kRed);
        }

        if (transitionTimer.isRunning()) {
            setBlink(Color.kPurple);
        }

        if (transitionTimer.hasElapsed(TIME_TO_BLINK / 2)) {
            transitionTimer.stop();
            transitionTimer.reset();
        }

        if (ourShiftTimer.isRunning()) {
            setBlink(Color.kDarkGreen);
        }

        if (ourShiftTimer.hasElapsed(TIME_TO_BLINK / 2)) {
            ourShiftTimer.stop();
            ourShiftTimer.reset();
        }

        if (theirShiftTimer.isRunning()) {
            setBlink(Color.kDarkBlue);
        }

        if (theirShiftTimer.hasElapsed(TIME_TO_BLINK / 2)) {
            theirShiftTimer.stop();
            theirShiftTimer.reset();
        }

        if (endGameTimer.isRunning()) {
            setBlink(Color.kYellow);
        }

        if (endGameTimer.hasElapsed(TIME_TO_BLINK / 2)) {
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

        if (isShiftEnded || shiftEndedTimer.isRunning()) {
            setColor(Color.kWhite);
            shiftEndedTimer.start();
            isShiftEnded = false;
        }

        if (shiftEndedTimer.hasElapsed(1)) {
            shiftEndedTimer.stop();
            shiftEndedTimer.reset();
        }

        if (userButtonTimer.isRunning()) {
            setBlink(Color.kOrange);
        }

        if (userButtonTimer.hasElapsed(1)) {
            userButtonTimer.stop();
            userButtonTimer.reset();
        }

        if (isQuestDisconnected) {
            // setBlink(Color.kDarkRed);
        }
    }
}

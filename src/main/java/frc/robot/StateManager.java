package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotCommon.RobotStates;
import frc.robot.RobotCommon.Shifts;

public class StateManager extends SubsystemBase {

    private static StateManager instance;

    public static StateManager getInstance() {
        if (instance == null) instance = new StateManager();
        return instance;
    }

    public static void setInstance(StateManager instance) {
        StateManager.instance = instance;
    }

    public boolean isStateChangeActivated;

    private Timer shiftTimer;
    private boolean isRedWonAuto;
    private int shiftNum;

    private StateManager() {
        this.isStateChangeActivated = false;

        RobotCommon.setShift(Shifts.Disable);
        this.isRedWonAuto = true;
        this.shiftTimer = new Timer();
        this.shiftNum = -1;

        setName("State Manager");
        SmartDashboard.putData(this);
    }

    @Override
    @SuppressWarnings("resource")
    public void initSendable(SendableBuilder builder) {
        SendableChooser<RobotStates> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Idle2", RobotStates.Idle);
        for (int i = 0; i < RobotStates.values().length; i++) {
            chooser.addOption(RobotStates.values()[i].name(), RobotStates.values()[i]);
        }
        chooser.onChange(this::manualStateChange);
        chooser.initSendable(builder);
        builder.addBooleanProperty("Is Activated", this::isStateChangeActivated, this::setStateChangeActivated);
        builder.addDoubleProperty("Time left", this::getTimeLeft, null);
        builder.addStringProperty("current state", () -> RobotCommon.getState().name(), null);
        builder.addStringProperty("current Shift", () -> RobotCommon.getShift().name(), null);
        builder.addStringProperty("next Shift", () -> RobotCommon.getNextShift().name(), null);
    }

    public void checkGameData() {
        if (DriverStation.getGameSpecificMessage() != "") {
            switch (DriverStation.getGameSpecificMessage()) {
                case "R":
                    isRedWonAuto = true;
                    if (shiftNum == 1)
                        RobotCommon.setNextShift(RobotCommon.isRed() ? Shifts.Inactive : Shifts.Active);
                    break;

                case "B":
                    isRedWonAuto = false;
                    if (shiftNum == 1)
                        RobotCommon.setNextShift(RobotCommon.isRed() ? Shifts.Active : Shifts.Inactive);
                    break;

                default:
                    break;
            }
        }
    }

    @Override
    public void periodic() {
        checkGameData();
        updateShift();

        if (isStateChangeActivated) {
            if (isOnTrench(true))
                RobotCommon.setState(RobotStates.Trench);
            else if (RobotCommon.getState().equals(RobotStates.Trench) && isOnTrench(false))
                return;
            else if (isHub()) {
                RobotCommon.setState(RobotStates.Hub);
            } else if (isDelivery()) {
                RobotCommon.setState(RobotStates.Delivery);
            } else {
                RobotCommon.setState(RobotStates.DriveWithIntake);
            }
        }
    }

    public boolean isStateChangeActivated() {
        return isStateChangeActivated;
    }

    public void setStateChangeActivated(boolean isActivated) {
        this.isStateChangeActivated = isActivated;
    }

    public double getTimeLeft() {
        switch (shiftNum) {
            case 0:
                return 20 - shiftTimer.get();
            case 1:
                return 10 - shiftTimer.get();
            case 6:
                return 30 - shiftTimer.get();

            default:
                return 25 - shiftTimer.get();

        }
    }

    public void resetShift() {
        if (shiftNum != 0) {
            shiftTimer.stop();
            shiftTimer.reset();
            shiftNum = -1;
            RobotCommon.changeShift(Shifts.Disable, Shifts.Auto);
        }
    }

    private void updateShift() {
        if (RobotCommon.getShift().equals(Shifts.Disable) && RobotState.isEnabled() && RobotState.isAutonomous()) {
            RobotCommon.changeShift(Shifts.Auto, Shifts.Transition);
            shiftTimer.start();
            shiftNum = 0;
        } else if (RobotCommon.getShift().equals(Shifts.Auto) && RobotState.isTeleop()) {
            RobotCommon.changeShift(Shifts.Transition,
                    isRedWonAuto && RobotCommon.isRed() ? Shifts.Inactive : Shifts.Active);
            shiftNum = 1;
            shiftTimer.reset();
            shiftTimer.start();
        } else if (RobotCommon.getShift().equals(Shifts.Transition) && shiftTimer.hasElapsed(10)) {
            RobotCommon.changeShift(isRedWonAuto && RobotCommon.isRed() ? Shifts.Inactive : Shifts.Active,
                    isRedWonAuto && RobotCommon.isRed() ? Shifts.Active : Shifts.Inactive);
            shiftTimer.reset();
            shiftNum = 2;
        } else if (RobotCommon.getShift().equals(Shifts.Active) && shiftTimer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.changeShift(Shifts.Inactive, shiftNum != 4 ? Shifts.Active : Shifts.Endgame);
            shiftTimer.reset();
            shiftNum++;
        } else if (RobotCommon.getShift().equals(Shifts.Inactive) && shiftTimer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.changeShift(Shifts.Active, shiftNum != 4 ? Shifts.Inactive : Shifts.Endgame);
            shiftTimer.reset();
            shiftNum++;
        } else if (shiftNum == 5 && shiftTimer.hasElapsed(25)) {
            RobotCommon.changeShift(Shifts.Endgame, Shifts.Disable);
            shiftNum = 6;
            shiftTimer.reset();
        }
    }

    public void manualStateChange(RobotStates state) {
        RobotCommon.setState(state);
        isStateChangeActivated = false;
    }

    private final double TRENCH_OFFSET = 0.5;

    private boolean isOnTrench(boolean isFuture) {
        Pose2d checkPose = isFuture ? RobotCommon.getFutureRobotPose() : RobotCommon.getCurrentRobotPose();
        return (checkPose.getY() > Field.TrenchRedAudience.Y_FRONT - 0.4
                || checkPose.getY() < Field.TrenchRedScoring.Y_BACK + 0.4)
                && ((checkPose.getX() < Field.HubRed.X_BACK + TRENCH_OFFSET
                        && checkPose.getX() > Field.HubRed.X_FRONT - TRENCH_OFFSET)
                        || (checkPose.getX() < Field.HubBlue.X_FRONT + TRENCH_OFFSET
                                && checkPose.getX() > Field.HubBlue.X_BACK - TRENCH_OFFSET));
    }

    private boolean isHub() {
        if (RobotCommon.isRed())
            return Field.HubRed.X_BACK < RobotCommon.getCurrentRobotPose().getX();
        else
            return Field.HubBlue.X_BACK > RobotCommon.getCurrentRobotPose().getX();
    }

    private boolean isDelivery() {
        if (RobotCommon.isRed())
            return Field.HubRed.X_FRONT >= RobotCommon.getCurrentRobotPose()
                    .getX();
        else
            return Field.HubBlue.X_FRONT <= RobotCommon.getCurrentRobotPose().getX();
    }

    public Timer getShiftTimer() {
        return shiftTimer;
    }

    public void setShiftTimer(Timer timer) {
        this.shiftTimer = timer;
    }

    public boolean isRedWonAuto() {
        return isRedWonAuto;
    }

    public void setRedWonAuto(boolean isRedWonAuto) {
        this.isRedWonAuto = isRedWonAuto;
    }

    public int getShiftNum() {
        return shiftNum;
    }

    public void setShiftNum(int shiftNum) {
        this.shiftNum = shiftNum;
    }
}

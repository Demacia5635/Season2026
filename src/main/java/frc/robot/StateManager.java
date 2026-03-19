package frc.robot;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.RobotCommon.Shifts;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

public class StateManager extends SubsystemBase {

    private static StateManager instance;

    public static void initialize(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinuaSubsystem,
            Turret turret,
            Shooter shooter, CommandController driverController, LedStrip leds) {
        if (instance == null)
            instance = new StateManager(chassis, intake, shinuaSubsystem, turret, shooter, driverController, leds);
    }

    public static StateManager getInstance() {
        return instance;
    }

    public static void setInstance(StateManager instance) {
        StateManager.instance = instance;
    }

    private final Chassis chassis;

    private final IntakeSubsystem intake;
    private final ShinuaSubsystem shinuaSubsystem;

    private final Turret turret;
    private final Shooter shooter;
    private final CommandController driverController;

    private final LedStrip leds;

    public boolean isStateChangeActivated;

    public boolean isAutoIntakeManual;
    private Timer timer;
    private boolean isRedWonAuto;
    private int shiftNum;

    private Timer intakeCurrentTimer;

    private StateManager(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinuaSubsystem, Turret turret,
            Shooter shooter, CommandController driverController, LedStrip ledStrip) {
        this.chassis = chassis;
        this.intake = intake;
        this.shinuaSubsystem = shinuaSubsystem;
        this.turret = turret;
        this.shooter = shooter;

        this.driverController = driverController;
        this.leds = ledStrip;

        this.isStateChangeActivated = false;
        this.isAutoIntakeManual = false;

        RobotCommon.currentShift = Shifts.Disable;
        this.isRedWonAuto = true;
        this.timer = new Timer();
        this.shiftNum = -1;

        this.intakeCurrentTimer = new Timer();

        setName("State Manager");
        SmartDashboard.putData(this);
    }

    public boolean isAutoIntakeManual() {
        return isAutoIntakeManual;
    }

    public void setAutoIntakeManual(boolean isAutoIntakeManual) {
        this.isAutoIntakeManual = isAutoIntakeManual;
    }

    @Override
    @SuppressWarnings("resource")
    public void initSendable(SendableBuilder builder) {
        SendableChooser<RobotStates> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Idle2", RobotStates.Idle);
        for (int i = 0; i < RobotStates.values().length; i++) {
            chooser.addOption(RobotStates.values()[i].name(), RobotStates.values()[i]);
        }
        chooser.onChange(this::onChange);
        chooser.initSendable(builder);
        builder.addBooleanProperty("Is Activated", this::isStateChangeActivated, this::setStateChangeActivated);
        builder.addDoubleProperty("Time left", this::getTimeLeft, null);
        builder.addStringProperty("current state", () -> RobotCommon.currentState.name(), null);
        builder.addStringProperty("current Shift", () -> RobotCommon.currentShift.name(), null);
    }

    public boolean isCanIntake() {
        intakeCurrentTimer.start();
        return intake.canIntake();
    }

    public void checkGameData() {
        if (DriverStation.getGameSpecificMessage() != "") {
            switch (DriverStation.getGameSpecificMessage()) {
                case "R":
                    isRedWonAuto = true;
                    if (shiftNum == 1)
                        RobotCommon.nextShift = RobotCommon.isRed ? Shifts.Inactive : Shifts.Active;
                    break;

                case "B":
                    isRedWonAuto = false;
                    if (shiftNum == 1)
                        RobotCommon.nextShift = RobotCommon.isRed ? Shifts.Active : Shifts.Inactive;
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
                RobotCommon.changeState(RobotStates.Trench);
            else if (RobotCommon.currentState == RobotStates.Trench && isOnTrench(false))
                return;
            else if (isHub()) {
                RobotCommon.changeState(RobotStates.Hub);
            } else if (isDelivery()) {
                RobotCommon.changeState(RobotStates.Delivery);
            } else {
                RobotCommon.changeState(RobotStates.DriveWithIntake);
            }
        }
    }

    public Chassis getChassis() {
        return chassis;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public ShinuaSubsystem getShinuaSubsystem() {
        return shinuaSubsystem;
    }

    public Turret getTurret() {
        return turret;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public CommandController getDriverController() {
        return driverController;
    }

    public LedStrip getLeds() {
        return leds;
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
                return 20 - timer.get();
            case 1:
                return 10 - timer.get();
            case 6:
                return 30 - timer.get();

            default:
                return 25 - timer.get();

        }
    }

    public void resetShift() {
        if (shiftNum != 0) {
            timer.stop();
            timer.reset();
            shiftNum = -1;
            RobotCommon.changeShift(Shifts.Disable, Shifts.Auto);
        }
    }

    private void updateShift() {
        if (RobotCommon.currentShift == Shifts.Disable && RobotState.isEnabled() && RobotState.isAutonomous()) {
            RobotCommon.changeShift(Shifts.Auto, Shifts.Transition);
            timer.start();
            shiftNum = 0;
        } else if (RobotCommon.currentShift == Shifts.Auto && RobotState.isTeleop()) {
            RobotCommon.changeShift(Shifts.Transition,
                    isRedWonAuto && RobotCommon.isRed ? Shifts.Inactive : Shifts.Active);
            shiftNum = 1;
            timer.reset();
            timer.start();
        } else if (RobotCommon.currentShift == Shifts.Transition && timer.hasElapsed(10)) {
            RobotCommon.changeShift(isRedWonAuto && RobotCommon.isRed ? Shifts.Inactive : Shifts.Active,
                    isRedWonAuto && RobotCommon.isRed ? Shifts.Active : Shifts.Inactive);
            timer.reset();
            shiftNum = 2;
        } else if (RobotCommon.currentShift == Shifts.Active && timer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.changeShift(Shifts.Inactive, shiftNum != 4 ? Shifts.Active : Shifts.Endgame);
            timer.reset();
            shiftNum++;
        } else if (RobotCommon.currentShift == Shifts.Inactive && timer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.changeShift(Shifts.Active, shiftNum != 4 ? Shifts.Inactive : Shifts.Endgame);
            timer.reset();
            shiftNum++;
        } else if (shiftNum == 5 && timer.hasElapsed(25)) {
            RobotCommon.changeShift(Shifts.Endgame, Shifts.Disable);
            shiftNum = 6;
            timer.reset();
        }
    }

    private void onChange(RobotStates state) {
        RobotCommon.changeState(state);
        isStateChangeActivated = false;
    }

    private final double TRENCH_OFFSET = 0.5;

    private boolean isOnTrench(boolean isFuture) {
        Pose2d checkPose = isFuture ? RobotCommon.futureRobotPose : RobotCommon.currentRobotPose;
        return (checkPose.getY() > Field.TrenchRedAudience.Y_FRONT - 0.4
                || checkPose.getY() < Field.TrenchRedScoring.Y_BACK + 0.4)
                && ((checkPose.getX() < Field.HubRed.X_BACK + TRENCH_OFFSET
                        && checkPose.getX() > Field.HubRed.X_FRONT - TRENCH_OFFSET)
                        || (checkPose.getX() < Field.HubBlue.X_FRONT + TRENCH_OFFSET
                                && checkPose.getX() > Field.HubBlue.X_BACK - TRENCH_OFFSET));
    }

    private boolean isAutoIntake() {
        if (RobotCommon.fuelPosition == null || (driverController.getLeftX() == 0 && driverController.getLeftY() == 0))
            return false;
        Translation2d controllerDirection = new Translation2d(driverController.getLeftX(),
                -driverController.getLeftY());
        return Math.abs(
                RobotCommon.fuelPosition.rotateBy(RobotCommon.robotAngle).getAngle().getRadians()
                        - controllerDirection.getAngle().getRadians()) < Math
                                .toRadians(27)
                && driverController.getLeftTrigger() < 0.2 && driverController.getRightTrigger() < 0.2;
    }

    private boolean isClimb() {
        return !DriverStation.isAutonomous()
                && ((RobotCommon.currentRobotPose.getX() < 10 && RobotCommon.currentRobotPose.getX() > 10)
                        && (RobotCommon.currentRobotPose.getY() < 10 && RobotCommon.currentRobotPose.getY() > 10));
    }

    private boolean isHub() {
        if (RobotCommon.isRed)
            return Field.HubRed.X_BACK < RobotCommon.currentRobotPose.getX();
        else
            return Field.HubBlue.X_BACK > RobotCommon.currentRobotPose.getX();
    }

    private boolean isDelivery() {
        if (RobotCommon.isRed)
            return Field.HubRed.X_FRONT >= RobotCommon.currentRobotPose
                    .getX();
        else
            return Field.HubBlue.X_FRONT <= RobotCommon.currentRobotPose.getX();
    }

    public Timer getTimer() {
        return timer;
    }

    public void setTimer(Timer timer) {
        this.timer = timer;
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

    public Timer getIntakeCurrentTimer() {
        return intakeCurrentTimer;
    }

    public void setIntakeCurrentTimer(Timer intakeCurrentTimer) {
        this.intakeCurrentTimer = intakeCurrentTimer;
    }
}

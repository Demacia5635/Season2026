package frc.robot;

<<<<<<< HEAD
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
=======
import edu.wpi.first.math.geometry.Translation2d;
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
=======
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.RobotCommon.Shifts;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660

public class StateManager extends SubsystemBase {

    private static StateManager instance;
<<<<<<< HEAD
    private boolean wantToAutoIntake = true;
    private Chassis chassis;
    private Shooter shooter;
    private Turret turret;
    private Rectangle2d trenchArea;
=======

    public static void initalize(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinuaSubsystem,
            Turret turret,
            Shooter shooter, CommandController driverController, LedStrip leds) {
        if (instance == null)
            instance = new StateManager(chassis, intake, shinuaSubsystem, turret, shooter, driverController, leds);
    }
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660

    public static StateManager getInstance() {
        return instance;
    }

<<<<<<< HEAD
    public void setWantToAutoIntake() {
        this.wantToAutoIntake = !wantToAutoIntake;
    }

    private RobotCommon.RobotStates lastState;

    private StateManager() {
        lastState = RobotCommon.RobotStates.IDLE;
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        this.trenchArea = RobotCommon.isRed ? .getRedTrenchArea() : Field.getInstance().getBlueTrenchArea();
    
=======
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
        this.shiftNum = 0;

        this.intakeCurrentTimer = new Timer();

        setName("State Manager");
        SmartDashboard.putData(this);
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660
    }

    public boolean isAutoIntakeManual() {
        return isAutoIntakeManual;
    }

<<<<<<< HEAD
    private void setWantedState(RobotCommon.RobotStates newState) {
        RobotCommon.changeState(newState);
    }

    private boolean isInCenter(){

        return RobotCommon.isRed ? chassis.getPose().getTranslation().getX() > trenchArea.getCenter().getTranslation().
    }

    private boolean isGoingToTrench() {
        return trenchArea.contains(chassis.getPoseWithVelocity(0.5).getTranslation());
    }

    private boolean isAtAlliance(){
        return RobotCommon.isRed ? chassis.getPose().getTranslation().getX() < Field.Red.ALLIANCE_LINE.getCenter().getTranslation().getX()
        : chassis.getPose().getTranslation().getX() > Field.Blue.ALLIANCE_LINE.getCenter().getTranslation().getX();
=======
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
        builder.addBooleanProperty("Is Auto Intake Activated", this::isAutoIntakeManual, this::setAutoIntakeManual);
        builder.addDoubleProperty("Time left", this::getTimeLeft, null);
        builder.addStringProperty("current state", () -> RobotCommon.currentState.name(), null);
    }

    public boolean isCanIntake() {
        intakeCurrentTimer.start();
        return intake.isCanIntake();
    }

    public void checkGameData() {
        if (DriverStation.getGameSpecificMessage() != "") {
            switch (DriverStation.getGameSpecificMessage()) {
                case "R":
                    isRedWonAuto = true;
                    break;

                case "B":
                    isRedWonAuto = false;
                    break;

                default:
                    break;
            }
        }
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660
    }

    @Override
    public void periodic() {
<<<<<<< HEAD
        if (lastState != RobotStates.Trench && isGoingToTrench()) {
            setWantedState(RobotStates.Trench);
            lastState = RobotStates.Trench;
            return;
        }
=======
        checkGameData();
        updateShift();

        if (isStateChangeActivated) {
            if (isOnTrench())
                RobotCommon.changeState(RobotStates.Trench);
            // else if (isClimb())
            // RobotCommon.changeState(RobotStates.Climb);
            if (!isCanIntake() && intakeCurrentTimer.hasElapsed(0.3))
                RobotCommon.hasDisabledIntake = true;
            if (isAutoIntakeManual && isAutoIntake())
                if (isHub()) {
                    RobotCommon.hasDisabledIntake = false;
                    RobotCommon.changeState(RobotStates.HubWithAutoIntake);
                } else if (isDelivery()) {
                    RobotCommon.hasDisabledIntake = false;
                    RobotCommon.changeState(RobotStates.DeliveryWithAutoIntake);
                } else
                    RobotCommon.changeState(RobotStates.DriveAutoIntake);
            else if (isHub()) {
                RobotCommon.hasDisabledIntake = false;
                RobotCommon.changeState(RobotStates.HubWithoutAutoIntake);
            } else if (isDelivery()) {
                RobotCommon.hasDisabledIntake = false;
                RobotCommon.changeState(RobotStates.DeliveryWithoutAutoIntake);
            } else
                RobotCommon.changeState(RobotStates.DriveWithIntake);
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

    private double getTimeLeft() {
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

    private void updateShift() {
        if (RobotCommon.currentShift == Shifts.Disable && RobotState.isEnabled() && RobotState.isAutonomous()) {
            RobotCommon.changeShift(Shifts.Auto);
            timer.start();
            shiftNum = 0;
        } else if (RobotCommon.currentShift == Shifts.Auto && RobotState.isTeleop()) {
            RobotCommon.changeShift(Shifts.Transition);
            shiftNum = 1;
            timer.reset();
            timer.start();
        } else if (RobotCommon.currentShift == Shifts.Transition && timer.hasElapsed(10)) {
            RobotCommon.changeShift(isRedWonAuto && RobotCommon.isRed ? Shifts.Active : Shifts.Inactive);
            timer.reset();
            shiftNum = 2;
        } else if (RobotCommon.currentShift == Shifts.Active && timer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.currentShift = Shifts.Inactive;
            timer.reset();
            shiftNum++;
        } else if (RobotCommon.currentShift == Shifts.Inactive && timer.hasElapsed(25) && shiftNum != 5) {
            RobotCommon.currentShift = Shifts.Active;
            timer.reset();
            shiftNum++;
        } else if (shiftNum == 5 && timer.hasElapsed(25)) {
            RobotCommon.currentShift = Shifts.Endgame;
            shiftNum = 6;
            timer.reset();
        }
    }

    private void onChange(RobotStates state) {
        RobotCommon.changeState(state);
        isStateChangeActivated = false;
    }

    private boolean isOnTrench() {
        return (RobotCommon.futureRobotPose.getX() > Field.TrenchRedAudience.X_FRONT
                && RobotCommon.futureRobotPose.getX() < Field.TrenchRedAudience.X_BACK)
                || (RobotCommon.currentRobotPose.getX() > Field.TrenchRedAudience.X_FRONT
                        && RobotCommon.currentRobotPose.getX() < Field.TrenchRedAudience.X_BACK)
                || (RobotCommon.futureRobotPose.getX() < Field.TrenchBlueAudience.X_FRONT
                        && RobotCommon.futureRobotPose.getX() > Field.TrenchBlueAudience.X_BACK)
                || (RobotCommon.currentRobotPose.getX() < Field.TrenchBlueAudience.X_FRONT
                        && RobotCommon.currentRobotPose.getX() > Field.TrenchBlueAudience.X_BACK);
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
        /* TODO: need to check climb position */
        return !DriverStation.isAutonomous();
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
>>>>>>> 8ff21cabe5c5ee54e6c4b85728e09fcc6406e660
    }
}

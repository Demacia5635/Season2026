package frc.robot;

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

    public static void initalize(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinuaSubsystem,
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
        chooser.setDefaultOption("Idle", RobotStates.Idle);
        for (int i = 1; i < RobotStates.values().length; i++) {
            chooser.addOption(RobotStates.values()[i].name(), RobotStates.values()[i]);
        }
        chooser.onChange(this::onChange);
        chooser.initSendable(builder);
        builder.addBooleanProperty("Is Activated", this::isStateChangeActivated, this::setStateChangeActivated);
        builder.addBooleanProperty("Is Auto Intake Activated", this::isAutoIntake, this::setAutoIntakeManual);
        builder.addDoubleProperty("Time left", this::getTimeLeft, null);
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


    public boolean isCanIntake(){
        return intake.isCanIntake();
    }

    @Override
    public void periodic() {
        updateShift();

        if (isStateChangeActivated) {
            if (isOnTrench())
                RobotCommon.changeState(RobotStates.Trench);
            else if (isClimb())
                RobotCommon.changeState(RobotStates.Climb);
            else if (isAutoIntakeManual && isAutoIntake())
                if(isCanIntake())
                    if (isHub())
                        RobotCommon.changeState(RobotStates.HubWithAutoIntake);
                    else if (isDelivery())
                        RobotCommon.changeState(RobotStates.DeliveryWithAutoIntake);
                    else
                        RobotCommon.changeState(RobotStates.DriveAutoIntake);
                else if (isHub())
                    RobotCommon.changeState(RobotStates.HubWithoutAutoIntake);
                else if (isDelivery())
                    RobotCommon.changeState(RobotStates.DeliveryWithoutAutoIntake);
                else
                    RobotCommon.changeState(RobotStates.DriveWithIntake);
                else
                    if(isHub())
                        RobotCommon.changeState(RobotStates.Hub);
                    else if(isDelivery())
                        RobotCommon.changeState(RobotStates.Delivery);
                    else
                        RobotCommon.changeState(RobotStates.Drive);
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

    private void onChange(RobotStates state) {
        RobotCommon.changeState(state);
        isStateChangeActivated = false;
    }

    private boolean isOnTrench() {
        return Field.LOWER_TRENCH(true).contains(RobotCommon.futureRobotPose.getTranslation())
                || Field.UPPER_TRENCH(true).contains(RobotCommon.futureRobotPose.getTranslation())
                || Field.LOWER_TRENCH(false).contains(RobotCommon.futureRobotPose.getTranslation())
                || Field.UPPER_TRENCH(false).contains(RobotCommon.futureRobotPose.getTranslation());

    }

    private boolean isAutoIntake() {
        if (RobotCommon.fuelPosition == null)
            return false;
        Translation2d robotToFuel = RobotCommon.fuelPosition.minus(RobotCommon.currentRobotPose.getTranslation());
        Translation2d controllerDirection = new Translation2d(driverController.getLeftX(),
                -driverController.getLeftY());
        return Math.abs(robotToFuel.getAngle().getRadians() - controllerDirection.getAngle().getRadians()) < Math
                .toRadians(30);
    }

    private boolean isClimb() {
        return !DriverStation.isAutonomous()
                && Field.CLIMB(RobotCommon.isRed).contains(RobotCommon.futureRobotPose.getTranslation());
    }

    private boolean isHub() {
        if (RobotCommon.isRed)
            return Field.ALLIANCE_LINE(true).getCenter().getX() < RobotCommon.currentRobotPose.getX();
        else
            return Field.ALLIANCE_LINE(false).getCenter().getX() > RobotCommon.currentRobotPose.getX();
    }

    private boolean isDelivery() {
        if (RobotCommon.isRed)
            return Field.HUB(true).getCenter().getX() - Field.HUB(true).getXWidth() / 2 >= RobotCommon.currentRobotPose
                    .getX();
        else
            return Field.HUB(false).getCenter().getX()
                    + Field.HUB(false).getXWidth() / 2 <= RobotCommon.currentRobotPose.getX();
    }
}

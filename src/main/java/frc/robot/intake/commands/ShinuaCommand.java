package frc.robot.intake.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.log.LogManager;
import frc.robot.RobotCommon;
import frc.robot.intake.subsystems.ShinuaSubsystem;

/**
 * The Command that activates the {@link ShinuaSubsystem} based on the
 * {@link RobotCommon.RobotStates}
 */
public class ShinuaCommand extends Command {

    /** The Shinua subsystem */
    private final ShinuaSubsystem shinua;

    /** Test power for top */
    private double topPow;
    /** Test power for left */
    private double leftPow;
    /** Test power for right */
    private double rightPow;
    /** Test power for battery */
    private double batteryPow;

    private Timer stuckBallsTimer;
    private Timer hasStuckTimer;
    private Timer handleStuckBallsTimer;

    private Timer testTimer;

    /**
     * Creates a new ShinuaCommand
     * 
     * @param shinua the shinua subsytem of the Robot Container
     */
    public ShinuaCommand(ShinuaSubsystem shinua) {
        this.shinua = shinua;

        topPow = 0;
        leftPow = 0;
        rightPow = 0;
        batteryPow = 0;

        stuckBallsTimer = new Timer();
        hasStuckTimer = new Timer();
        handleStuckBallsTimer = new Timer();
        testTimer = new Timer();

        addRequirements(shinua);

        SmartDashboard.putData("Shinua Command", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Batter Power", this::getBatteryPow, this::setBatteryPow);
        builder.addDoubleProperty("Left Power", this::getLeftPow, this::setLeftPow);
        builder.addDoubleProperty("Right Power", this::getRightPow, this::setRightPow);
        builder.addDoubleProperty("Top Power", this::getTopPow, this::setTopPow);
    }

    @Override
    public void initialize() {
    }

    private void applyShootingValues() {

        shinua.setDutyIndexerClose(1);
        shinua.setDutyIndexerFar(1);
        shinua.setDutyIndexerOnTop(1);
        shinua.setPowerBattery(1);

    }

    private void applyIntakeValues() {
        shinua.setDutyIndexerClose(0.8);
        shinua.setDutyIndexerFar(-0.3);
        shinua.setDutyIndexerOnTop(-1);
        shinua.setPowerBattery(0);

    }

    private boolean isBallsStuck() {
        return false;
        // return shinua.getIndexerOnTopCurrent() > 18
        // && Math.abs(shinua.getIndexerOnTopVelocity()) < 35;
    }

    private void handleBallsStuck() {
        shinua.setDutyIndexerFar(-1);
        shinua.setDutyIndexerClose(-1);
        shinua.setDutyIndexerOnTop(-1);
    }

    /**
     * 
     */
    @Override
    public void execute() {
        switch (RobotCommon.getState()) {
            case DriveWithIntake:
                applyIntakeValues();
                break;
            case Hub, Delivery:
                if (isBallsStuck() && !stuckBallsTimer.isRunning() && !hasStuckTimer.isRunning()){
                    stuckBallsTimer.reset();    
                    stuckBallsTimer.start();
                }

                if (stuckBallsTimer.isRunning() && !isBallsStuck()) {
                    LogManager.log(stuckBallsTimer.get());
                    stuckBallsTimer.stop();
                    stuckBallsTimer.reset();
                }

                if (isBallsStuck() && stuckBallsTimer.hasElapsed(0.15) && !hasStuckTimer.isRunning()) {
                    LogManager.log("stuck");
                    stuckBallsTimer.stop();
                    stuckBallsTimer.reset();
                    hasStuckTimer.reset();
                    hasStuckTimer.start();
                    RobotCommon.setStuck(true);
                    handleBallsStuck();
                    break;
                }

                if (hasStuckTimer.isRunning() && !hasStuckTimer.hasElapsed(0.2)) {
                    handleBallsStuck();
                    break;
                }

                if (hasStuckTimer.hasElapsed(0.1)) {
                    hasStuckTimer.stop();
                    hasStuckTimer.reset();
                    RobotCommon.setStuck(false);
                }

                if (RobotCommon.isReady()) {
                    applyShootingValues();
                } else {
                    applyIntakeValues();
                    shinua.setPowerBattery(0.1);
                }

                break;

            case Test:
                shinua.setDutyIndexerClose(rightPow);
                shinua.setDutyIndexerFar(leftPow);
                shinua.setDutyIndexerOnTop(topPow);
                shinua.setPowerBattery(batteryPow);
                break;

            default:
                shinua.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shinua.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public ShinuaSubsystem getShinua() {
        return shinua;
    }

    public double getTopPow() {
        return topPow;
    }

    public void setTopPow(double topPow) {
        this.topPow = topPow;
    }

    public double getLeftPow() {
        return leftPow;
    }

    public void setLeftPow(double leftPow) {
        this.leftPow = leftPow;
    }

    public double getRightPow() {
        return rightPow;
    }

    public void setRightPow(double rightPow) {
        this.rightPow = rightPow;
    }

    public double getBatteryPow() {
        return batteryPow;
    }

    public void setBatteryPow(double batteryPow) {
        this.batteryPow = batteryPow;
    }
}

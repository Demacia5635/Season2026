package frc.robot.intake.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.RobotCommon;
import frc.robot.intake.IntakeConstants;
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
    private Timer timer;
    private boolean hasStarted = false;

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

        shinua.setDutyIndexerClose(IntakeConstants.MAX_POWER);
        shinua.setDutyIndexerFar(IntakeConstants.MAX_POWER);
        shinua.setDutyIndexerOnTop(1);
        shinua.setPowerBattery(1);

    }

    private void applyIntakeValues() {
        shinua.setDutyIndexerClose(0.8);
        shinua.setDutyIndexerFar(0);
        shinua.setDutyIndexerOnTop(-0.8);
        shinua.setPowerBattery(0);

    }

    private boolean isBallsStuck() {

        return (shinua.getIndexerOnTopCurrent() > 18 && shinua.getIndexerOnTopVelocity() < Math.PI);
    }

    private void handleBallsStuck() {
        if (hasStarted) {
            if (timer.hasElapsed(0.25))
                hasStarted = false;
            else {

                shinua.setDutyIndexerFar(-0.4);
                shinua.setDutyIndexerClose(-0.4);
                shinua.setDutyIndexerOnTop(-1);

            }
        }
    }

    /**
     * 
     */
    @Override
    public void execute() {
        switch (RobotCommon.currentState) {
            case DriveWithIntake:
                applyIntakeValues();
                break;
            case Hub, Delivery:

                if (isBallsStuck()) {
                    handleBallsStuck();
                    return;
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

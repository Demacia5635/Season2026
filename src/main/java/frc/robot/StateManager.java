package frc.robot;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;

public class StateManager extends SubsystemBase {

    private static StateManager instance;
    private boolean wantToAutoIntake = true;
    private Chassis chassis;
    private Shooter shooter;
    private Turret turret;
    private Rectangle2d trenchArea;

    public static StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }
        return instance;
    }

    public void setWantToAutoIntake() {
        this.wantToAutoIntake = !wantToAutoIntake;
    }

    private RobotCommon.RobotStates lastState;

    private StateManager() {
        lastState = RobotCommon.RobotStates.IDLE;
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        this.trenchArea = RobotCommon.isRed ? .getRedTrenchArea() : Field.getInstance().getBlueTrenchArea();
    
    }

    public RobotCommon.RobotStates getLastState() {
        return lastState;
    }

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
    }

    @Override
    public void periodic() {
        if (lastState != RobotStates.Trench && isGoingToTrench()) {
            setWantedState(RobotStates.Trench);
            lastState = RobotStates.Trench;
            return;
        }
    }
}

package frc.robot.Turret.TurretCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.vision.TagPose;
import frc.robot.RobotCommon.robotStates;
import frc.robot.Turret.Turret;


public class TurretCommand extends Command{
    Turret turret;
    TagPose tag;
    robotStates state;
    double wantedAngle = 0;
    
    public TurretCommand(){
        this.turret = Turret.getInstance();
        addRequirements(turret);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Has Calibrated", ()->turret.hasCalibrated(), null);
        builder.addDoubleProperty("Wanted Turret Angle", ()->wantedAngle, (x)->wantedAngle = x);
    }

    @Override
    public void execute() {
        if(!turret.hasCalibrated() || !tag.isSeeTag(0,tag.GetDistFromCamera())) return;
        // turret.setPositionMotion(Math.toRadians(wantedAngle));
        switch(state){
            case ShootWithIntake,ShootWithoutIntake :
                turret.setPositionMotion(turret.getTurretToHubAngle());
                break;
            default:
                break;
        }
    }


}

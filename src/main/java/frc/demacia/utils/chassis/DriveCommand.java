// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import java.util.logging.LogManager;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.vision.ObjectPose;
import frc.demacia.vision.subsystem.Dvirs_ObjectPose;
import frc.robot.RobotCommon;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private Chassis chassis;
  private CommandController controller;
  private double direction;
  private ChassisSpeeds speeds;
  private boolean precisionMode;
  private Dvirs_ObjectPose objectPose;
  private double maxVelocityAutoIntake = 3;
  private final Translation2d chassisToIntakeOffset = new Translation2d(0.3, 0);

  /** Creates a new DriveCommand. */
  public DriveCommand(Chassis chassis, CommandController controller, Dvirs_ObjectPose objectPose) {
    this.chassis = chassis;
    this.controller = controller;
    precisionMode = false;
    this.objectPose = objectPose;
    addRequirements(chassis);
  }

  public void invertPrecisionMode() {
    setPrecisionMode(!precisionMode);
  }

  public void setActiveToHub() {
    chassis.setRotateToHub();
  }

  public void setPrecisionMode(boolean precisionMode) {
    this.precisionMode = precisionMode;
  }

  public boolean getPrecisionMode() {
    return precisionMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotCommon.currentState) {
      case HubWithAutoIntake, DeliveryWithAutoIntake:// , DriveAutoIntake:
        Translation2d driverVelocityVectorRobotRel = new Translation2d(controller.getLeftY(), controller.getLeftX())
            .rotateBy(chassis.getGyroAngle().unaryMinus());
        double wantedVxRobotRel = (Math.min(
            Math.abs(driverVelocityVectorRobotRel.getX() * chassis.getConfig().maxDriveVelocity),
            maxVelocityAutoIntake));
        Translation2d intakeToTarget = objectPose.giveBestTranslation().minus(chassisToIntakeOffset);
        double angleToFix = Math.min(Math.abs(intakeToTarget.getAngle().getRadians() * 2), Math.PI * 0.5)
            * Math.signum(intakeToTarget.getAngle().getRadians());

        ChassisSpeeds chassisWantSpeeds = new ChassisSpeeds(wantedVxRobotRel, -wantedVxRobotRel * Math.tan(angleToFix),
            0);

        chassis.setRobotRelVelocities(chassisWantSpeeds);
        frc.demacia.utils.log.LogManager
            .log("DIs: " + intakeToTarget.getNorm() + " angle: " + intakeToTarget.getAngle());
        break;
      default:
        direction = RobotCommon.isRed ? 1 : -1;
        double joyX = controller.getLeftY() * direction;
        double joyY = controller.getLeftX() * direction;

        // Calculate r]otation from trigger axes
        double rot = controller.getLeftTrigger() - controller.getRightTrigger();

        double velX = Math.pow(joyX, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyX);
        double velY = Math.pow(joyY, 2) * chassis.getConfig().maxDriveVelocity * Math.signum(joyY);
        double velRot = Math.pow(rot, 2) * chassis.getConfig().maxRotationalVelocity * Math.signum(rot);
        if (precisionMode) {
          velX /= 4;
          velY /= 4;
          velRot /= 4;
        }

        speeds = new ChassisSpeeds(velX, velY, -velRot);

        chassis.setVelocities(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

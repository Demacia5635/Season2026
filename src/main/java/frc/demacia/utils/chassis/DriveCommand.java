// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import java.util.logging.LogManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double maxVelocityAutoIntake = 3;
  private final Translation2d chassisToIntakeOffset = new Translation2d(0.3, 0);

  /** Creates a new DriveCommand. */
  public DriveCommand(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    precisionMode = false;
    SmartDashboard.putData("drive command", this);
    addRequirements(chassis);
  }

  private void driveByJoystick() {
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
    // System.out.println("Yaw"+objectPose.getYaw());
    // System.out.println("dist:"+objectPose.getDistance());
    // frc.demacia.utils.log.LogManager.log("Yaw"+objectPose.getYaw());
    // frc.demacia.utils.log.LogManager.log("dist:"+objectPose.getDistance());

    switch (RobotCommon.currentState) {
      case HubWithAutoIntake, DeliveryWithAutoIntake, DriveAutoIntake:
        if (RobotCommon.fuelPosition != null) {

          Translation2d driverVelocityVectorRobotRel = new Translation2d(controller.getLeftY(), controller.getLeftX())
              .rotateBy(chassis.getGyroAngle().unaryMinus());
          double wantedVxRobotRel = (Math.min(
              Math.abs(driverVelocityVectorRobotRel.getX() * chassis.getConfig().maxDriveVelocity),
              maxVelocityAutoIntake));
          Translation2d intakePosition = chassis.getPose().getTranslation()
              .plus(chassisToIntakeOffset.rotateBy(chassis.getGyroAngle()));
          // frc.demacia.utils.log.LogManager.log("intakePosition: " + intakePosition + "
          // fuel pos: " + RobotCommon.fuelPosition);

          // SmartDashboard.putNumber("intakePositionX: ", intakePosition.getX());
          // SmartDashboard.putNumber("intakePositionY: ", intakePosition.getY());
          // SmartDashboard.putNumber("intakePfuelX: ", RobotCommon.fuelPosition.getX());
          // SmartDashboard.putNumber("intakePfuelY: ", RobotCommon.fuelPosition.getY());
          Translation2d intakeToTarget = RobotCommon.fuelPosition.minus(intakePosition);
          double fuelDir = intakeToTarget.getAngle().minus(RobotCommon.robotAngle).getRadians();
          SmartDashboard.putNumber("FuelDir: ", Math.toDegrees(fuelDir));
          if (Math.abs(fuelDir) > Math.PI / 4) {
            fuelDir = 0;
          }
          ChassisSpeeds chassisWantSpeeds = new ChassisSpeeds(wantedVxRobotRel,
              wantedVxRobotRel * Math.tan(2 * fuelDir),
              0);
          frc.demacia.utils.log.LogManager.log("Angle to fix: " + fuelDir + " intake to target " + intakeToTarget
              + " wanted Vx robot rel " + wantedVxRobotRel + " angle to fix Abs wthout limit: "
              + (Math.abs(intakeToTarget.getAngle().getRadians() * 2)));
          SmartDashboard.putNumber("intakepAngletofix: ", fuelDir);
          SmartDashboard.putNumber("intakepttargetx: ", intakeToTarget.getX());
          SmartDashboard.putNumber("intakeptotargety: ", intakeToTarget.getY());
          // -1.5707963267948966 Translation2d(X: -1.16, Y: -1.56) 0.3934561183461096P6
          chassis.setRobotRelVelocities(chassisWantSpeeds);
          frc.demacia.utils.log.LogManager
              .log("DIs: " + intakeToTarget.getNorm() + " angle: " + intakeToTarget.getAngle());
        } else {
          driveByJoystick();
        }
        break;
      default:
        driveByJoystick();
        break;

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

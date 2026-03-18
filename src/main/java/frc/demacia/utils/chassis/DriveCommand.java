// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.ObjectPose;
import frc.demacia.vision.subsystem.Dvirs_ObjectPose;
import frc.robot.RobotCommon;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.intake.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private Chassis chassis;
  private CommandController controller;
  private double direction;
  private ChassisSpeeds speeds;
  private boolean precisionMode;
  private double maxVelocityAutoIntake = 3;
  private final Translation2d chassisToIntakeOffset = new Translation2d(0.25, -0.08);

  /** Creates a new DriveCommand. */
  public DriveCommand(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
     precisionMode = false;
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

    if (RobotCommon.currentState.equals(RobotCommon.RobotStates.Trench)
        && Shooter.getInstance().getHoodAngle() < Math.toRadians(80)) {
      velX /= 2;
      velY /= 2;
      // velX = Math.signum(velX) * Math.max(Math.abs(velX), 2);
      // velY = Math.signum(velY) * Math.max(Math.abs(velY), 2);
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
    // LogManager.log("Yaw"+objectPose.getYaw());
    // LogManager.log("dist:"+objectPose.getDistance());
    driveByJoystick();
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

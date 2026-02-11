// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.robot.RobotCommon;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateBasedClimb extends Command {
  /** Creates a new StateBasedClimb. */
  Chassis chassis;
  Climb climb;
  CommandController contoller;
  private boolean IS_AT_BAR;
  private boolean IS_AT_GROUND;
  private boolean IS_READY_TO_CLIMB;
  private boolean IS_RIGHT_CLIMB;
  private Pose2d chassisPose;
  private Translation2d diff;
  private ChassisSpeeds s;
  private double headingDiff;
  private Pose2d targetPose;

  public StateBasedClimb(Climb climb, CommandController controller, Chassis chassis) {
    this.climb = climb;
    this.chassis = chassis;
    this.contoller = controller;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IS_AT_BAR = false;
    IS_AT_GROUND = false;
    IS_RIGHT_CLIMB = true; // need to set based on strategy
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (RobotCommon.currentState) {
      case ShootWithIntake, ShootWithoutIntake, DriveWhileIntake, Drive:
        climb.armStateClose();
        break;
      case PrepareClimb:
        climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSED);
        if (climb.getArmEncoderAngle() >= ClimbConstants.ANGLE_ARMS_RAISED) {
          climb.stopArms();
        }
        targetPose = IS_RIGHT_CLIMB ? ClimbConstants.targetRightSide : ClimbConstants.targetLeftSide;
        chassisPose = chassis.getPose();
        diff = targetPose.getTranslation().minus(chassisPose.getTranslation());
        headingDiff = targetPose.getRotation().getRadians() - chassisPose.getRotation().getRadians();
        s = new ChassisSpeeds(diff.getX() * ClimbConstants.driveKp, diff.getY() * ClimbConstants.driveKp,
            headingDiff * ClimbConstants.rotationKp);
        chassis.setVelocities(s);
        break;
      case Climb:
        climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_LOWERED);
        if (climb.getArmEncoderAngle() >= ClimbConstants.ANGLE_ARMS_LOWERED) {
          climb.stopArms();
          IS_AT_BAR = true;
        }
        if (IS_AT_BAR) {
          chassisPose = chassis.getPose();
          diff = ClimbConstants.targetToFullyCloseArms.getTranslation().minus(chassisPose.getTranslation());
          s = new ChassisSpeeds(diff.getX() * ClimbConstants.driveKp, 0, 0);
          chassis.setVelocities(s);
        }
        if (chassisPose.getTranslation()
            .getDistance(ClimbConstants.targetToFullyCloseArms.getTranslation()) < ClimbConstants.CHASSIS_TOLERANCE) {
          IS_READY_TO_CLIMB = true;
        }

        if (IS_READY_TO_CLIMB) {
          climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_OPEN);
        }
        if (climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_OPEN) {
          climb.stopLever();
        }
        break;
      case GetOffClimb:
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSED);
        if (climb.getAngleLever() <= ClimbConstants.ANGLE_LEVER_CLOSED) {
          climb.stopLever();
          IS_AT_GROUND = true;
        }
        if (IS_AT_GROUND) {
          chassisPose = chassis.getPose();
          diff = ClimbConstants.targetToOpenArmsAfterClimb.getTranslation().minus(chassisPose.getTranslation());
          s = new ChassisSpeeds(diff.getX() * ClimbConstants.driveKp, 0, 0);
          chassis.setVelocities(s);
          climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
        }
        if (climb.getArmEncoderAngle() >= ClimbConstants.ANGLE_ARMS_RAISED) {
          climb.stopArms();
        }
        break;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

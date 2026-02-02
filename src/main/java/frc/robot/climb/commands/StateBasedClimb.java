// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import com.ctre.phoenix.motorcontrol.IFollower;

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
  private double joyright;
  private double joyleft;
  private boolean IS_AT_BAR;
  private boolean IS_AT_GROUND;
  private Pose2d targetRightSide = Pose2d.kZero;
  private Pose2d targetLeftSide = Pose2d.kZero;
  private double driveKp = 1.3;
  private double rotationKp = 2.2;

  public StateBasedClimb(Climb climb, CommandController controller,Chassis chassis) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (RobotCommon.currentState) {
      case ShootWithIntake, ShootWithoutIntake, DriveWhileIntake, Drive:
      if(climb.getArmsAngle() != ClimbConstants.ARMS_ANGLE_CLOSED   || climb.getAngleLever() != ClimbConstants.ANGLE_LEVER_CLOSE){
        climb.setArmsAngle(ClimbConstants.ARMS_ANGLE_CLOSED);
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
      }
        break;
      case PrepareClimb:
      Pose2d chassisPose = chassis.getPose();
    Translation2d diff = targetRightSide.getTranslation().minus(chassisPose.getTranslation());
    double headingDiff = targetRightSide.getRotation().getRadians() - chassisPose.getRotation().getRadians();
    ChassisSpeeds s = new ChassisSpeeds(diff.getX() * driveKp, diff.getY() * driveKp, headingDiff * rotationKp);
        climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
        if (climb.getArmsAngle() >= ClimbConstants.ANGLE_ARMS_RAISED) {
          climb.stopArms();
        }
        if (climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_CLOSE) {
          climb.stopLever();
        }
        break;
      case Climb:
      
        climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_LOWERED);
        if (climb.getArmsAngle() >= ClimbConstants.ANGLE_ARMS_LOWERED) {
          climb.stopArms();
          IS_AT_BAR = true;
        }
        if (IS_AT_BAR) {
          climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_OPEN);
        }
        if (climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_OPEN) {
          climb.stopLever();
        }

        break;
      case GetOffClimb:
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
        if (climb.getAngleLever() <= ClimbConstants.ANGLE_LEVER_CLOSE) {
          climb.stopLever();
          IS_AT_GROUND = true;
        }
        if (IS_AT_GROUND) {
          climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
        }
        if (climb.getArmsAngle() >= ClimbConstants.ANGLE_ARMS_RAISED) {
          climb.stopArms();
        }
        break;
      case TESTING:
        joyright = contoller.getRightY() * 0.2;
        climb.setArmsDuty(joyright);
        joyleft = contoller.getLeftY() * 0.2;
        climb.setLeverDuty(joyleft);
        break;
      case CLOSE:
        climb.setArmsAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
        if (climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_CLOSE)
          climb.stopLever();
        if (climb.getArmsAngle() >= ClimbConstants.ARMS_ANGLE_CLOSED)
          climb.stopLever();
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

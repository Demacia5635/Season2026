// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.geometry.ChassisSpeedsDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;
import frc.robot.Field;
import frc.robot.RobotCommon;
import frc.robot.Shooter.constants.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Shooter.utils.ShooterUtils;
import frc.robot.Turret.TurretConstants;

/**
 * this is the main shooter command
 * 
 * he cange the shooter by the state:
 * 
 * @param delevery            that the state for delvery the boll from the
 *                            mideal to are side
 * @param shooting            that state is for shooting boll to the hub
 * @param tranch              that state is for move ander the tranch
 * 
 *                            he cuod shoot and delvery by moving with the
 *                            funcsan
 * @param setShootingAndAngle that thke the wanted angle and vel and set it to
 *                            be wille moving
 * 
 *                            in the end of the command the shooter stop
 */

public class ShooterCommand extends Command {

  private final Shooter shooter;

  private double WHEEL_TO_BALL_VELOCITY_RATIO = 0.45;
  private double MAGNUS_CORRECTION = 0.2;
  private double velocityFromBattery = 1;

  private double wantedAngle;
  private double wantedVel;
  private double wantedFeederPower;

  private double HOOD_OFFSET = Math.toRadians(4);
  private double VELOCITY_CORRECTION = 1;

  public ShooterCommand(Shooter shooter) {
    this.shooter = shooter;
    this.wantedAngle = Math.toDegrees(shooter.getHoodAngleAbsEncoder());
    this.wantedVel = 0;
    this.wantedFeederPower = 0;

    addRequirements(shooter);
    SmartDashboard.putData("Shooter Command", this);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("wanted angle", () -> wantedAngle, (value) -> wantedAngle = value);
    builder.addDoubleProperty("wanted vel", () -> wantedVel, (value) -> wantedVel = value);
    builder.addDoubleProperty("wanted feeder power", () -> wantedFeederPower, (value) -> wantedFeederPower = value);
  }

  @Override
  public void initialize() {
  }

  private void setFlywheelAndHood(double lutVel, double lutHoodAngle, Rotation2dDemacia heading) {

    // set the horizontal (xy) velocity and the vertical (z) velocity
    double xyVel = lutVel * Math.cos(lutHoodAngle);
    double zVel = lutVel * Math.sin(lutHoodAngle);

    // calculate the x/y velocities correected by robot speeds
    double xVel = xyVel * heading.getCos() - robotSpeeds.vxMetersPerSecond * VELOCITY_CORRECTION;
    double yVel = xyVel * heading.getSin() - robotSpeeds.vyMetersPerSecond * VELOCITY_CORRECTION;

    xyVel = Math.hypot(xVel, yVel);
    // calculate the new ball velocity
    double ballVelocity = Math.hypot(xyVel, zVel);

    ballVelocity -= MAGNUS_CORRECTION * (ballVelocity - lutVel); // correct for Magnus (back spin) effect

    ballVelocity = ballVelocity / WHEEL_TO_BALL_VELOCITY_RATIO; // translate required ball velocity to flywheel
                                                                // velocity

    // calculate the hood angle
    double hoodAngle = Math.atan(zVel / xyVel) - HOOD_OFFSET; // with hood correction
    // check for max angle
    if (hoodAngle > ShooterConstans.MAX_ANGLE_HOOD) {
      hoodAngle = ShooterConstans.MAX_ANGLE_HOOD;
      ballVelocity = xyVel / Math.cos(hoodAngle);
    }

    // calculate the heading
    Rotation2dDemacia ballHeading = new Rotation2dDemacia(xVel, yVel);

    // LogManager.log("new hood angle: " + hoodAngle + " ball heading: " +
    // ballHeading);
    RobotCommon.setFutureAngleFromTargetRobotRelative(MathUtil
        .inputModulus(ballHeading.getRadians() - nextPose.getRotation().getRadians(), 0, 2 * Math.PI));
    shooter.setFlywheelVel(ballVelocity);
    shooter.setHoodAngle(hoodAngle);
  }

  ChassisSpeedsDemacia robotSpeeds;
  Pose2dDemacia nextPose;
  Translation2dDemacia toHub;
  Translation2dDemacia turretPos;
  double distance;
  double[] lut;

  @Override
  public void execute() {
    double vel = 0;
    double hoodAngle = 0;
    Rotation2dDemacia heading = Rotation2dDemacia.kZero;

    robotSpeeds = RobotCommon.getFieldRelativeSpeeds();
    nextPose = ShooterUtils.computeFuturePosition(RobotCommon.getFieldRelativeSpeeds(),
        RobotCommon.getCurrentRobotPose(),
        0.2);

    switch (RobotCommon.getState()) {
      case Delivery:
        Translation2dDemacia chassisToDelivery = ShooterUtils.getDeliveryPoint().minus(nextPose.getTranslation());
        hoodAngle = Math.toRadians(45);
        vel = Math.sqrt(chassisToDelivery.getNorm() * 9.81);
        heading = chassisToDelivery.getAngle();
        setFlywheelAndHood(vel, hoodAngle, heading);
        if (RobotCommon.isReady())
          shooter.setFeederPower(0.6);
        break;

      case Hub:
        turretPos = nextPose.getTranslation()
            .plus(TurretConstants.TURRET_POSITION_ON_ROBOT.rotateBy(RobotCommon.getRobotAngle()));
        toHub = Field.HubRed.CENTER.minus(turretPos);

        // get the distance, heading and LUT valuse
        distance = toHub.getNorm();
        heading = toHub.getAngle();

        RobotCommon.setCurrentDistanceFromTarget(distance);

        lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
        double lutVel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO * velocityFromBattery; // correct to actual ball shooting
        double lutHoodAngle = lut[1] + HOOD_OFFSET; // correct to actual ball pitch

        if (RobotCommon.isReady())
          shooter.setFeederPower(0.7);
        setFlywheelAndHood(lutVel, lutHoodAngle, heading);
        break;

      case Trench:
        shooter.setHoodAngle(Math.toRadians(90));
        shooter.setFlywheelVel(10);
        break;

      case DriveWithIntake:
        shooter.setHoodAngle(Math.toRadians(90));
        shooter.setFlywheelPower(0);
        shooter.setFeederPower(0);
        break;

      case Test:
        shooter.setHoodAngle(Math.toRadians(wantedAngle));
        shooter.setFlywheelVel(wantedVel);
        shooter.setFeederPower(wantedFeederPower);
        break;

      default:
        shooter.setHoodPower(0);
        shooter.setFlywheelPower(0);
        shooter.setFeederPower(0);
        break;
    }
  }

  /**
   * the funcsan @param end stop the shooter when the command end
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

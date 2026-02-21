// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.robot.Field;
import frc.robot.RobotCommon;
import frc.robot.Shooter.constants.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Shooter.utils.ShooterUtils;
import frc.robot.Turret.Turret;

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
  Shooter shooter;
  Chassis chassis;
  private double WHEEL_TO_BALL_VELOCITY_RATIO = 0.51;
  private double MAGNUS_CORRECTION = 0.2;
  private double wantedAngle;
  private double wantedVel;
  private double velocityFromBattery = 1;

  private double HOOD_OFFSET = Math.toRadians(9);
  private double VELOCITY_CORRECTION = 1;
  private boolean shootVelocityWasOK = false;

  public ShooterCommand(Shooter shooter, Chassis chassis) {
    this.chassis = chassis;
    this.shooter = shooter;
    this.wantedAngle = Math.toDegrees(shooter.getHoodAngle());
    this.wantedVel = 0;

    addRequirements(shooter);
    SmartDashboard.putData(this);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("wanted angle", () -> wantedAngle, (value) -> wantedAngle = value);
    builder.addDoubleProperty("wanted vel", () -> wantedVel, (value) -> wantedVel = value);
  }
  @Override
  public void initialize() {
  }

  ChassisSpeeds robotSpeeds;
  Pose2d nextPose;
  Translation2d toHub;
  Translation2d turretPos;
  double distance;
  double[] lut;

  /**
   * this function the execute run every 0.02 secand
   * he set what angle and vel the shooter shood be by the state
   * 
   * @param shooter.getCurrentShooterState() this is how we get witch state the
   *                                         shooter need to be
   * 
   *                                         the state are:
   * 
   * @param delivery                         this state is for delever the code
   *                                         from the senter of the field to are
   *                                         sidr team
   * @param shooting                         this state is to shoot the fuol to
   *                                         the hub
   * @param tranch                           this state is to move the hood to
   *                                         angle for get ander the truench
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = 0;
    double hoodAngle = 0;
    Rotation2d heading = Rotation2d.kZero;
    switch (shooter.getCurrentShooterState()) {
      case DELIVERY:
        // Translation2d chassisToDelivery = RobotCommon.deliveryTarget
        // .minus(RobotContainer.chassis.getPose().getTranslation());
        hoodAngle = Math.toRadians(45);
        // vel = 1.7 * Math.sqrt(chassisToDelivery.getNorm() * 9.81);
        // heading = chassisToDelivery.getAngle();

        break;

      case SHOOTING_WITH_MOVEMENT:
        robotSpeeds = RobotCommon.fieldRelativeSpeeds;
        nextPose = ShooterUtils.computeFuturePosition(RobotCommon.fieldRelativeSpeeds, RobotCommon.currentRobotPose, 0.07);
        turretPos = nextPose.getTranslation()
            .plus(ShooterConstans.TURRET_POSITION_ON_ROBOT.rotateBy(chassis.getPose().getRotation()));
        toHub = Field.HUB(true).getCenter().getTranslation().minus(turretPos);

        // get the distance, heading and LUT valuse
        distance = toHub.getNorm();
        heading = toHub.getAngle();

        lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
        double lutVel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO * velocityFromBattery; // correct to actual ball shooting
        double lutHoodAngle = lut[1] + HOOD_OFFSET; // correct to actual ball pitch

        // set the horizontal (xy) velocity and the vertical (z) velocity
        double xyVel = lutVel * Math.cos(lutHoodAngle);
        double zVel = lutVel * Math.sin(lutHoodAngle);

        // calculate the x/y velocities correected by robot speeds
        double xVel = xyVel * heading.getCos() - robotSpeeds.vxMetersPerSecond * VELOCITY_CORRECTION;
        double yVel = xyVel * heading.getSin() - robotSpeeds.vyMetersPerSecond * VELOCITY_CORRECTION;

        xyVel = Math.hypot(xVel, yVel);
        // calculate the new ball velocity
        double ballVelocity = Math.hypot(xyVel, zVel);

        // LogManager.log("Distance from hub: " + distance + " heading to hub: " +
        // heading + " LUT vel: " + lutVel + " LUT hood angle: " + lutHoodAngle + " LUT
        // ball xyVel: " + xyVel
        // + " Z vel: " + zVel + " x Vel: " + xVel + " yVel: " + yVel + " newVel: " +
        // xyVel
        // + " ball vel pre magnus: " + ballVelocity);
        ballVelocity -= MAGNUS_CORRECTION * (ballVelocity - lutVel); // correct for Magnus (back spin) effect

        // LogManager.log("ball vel after magnus: " + ballVelocity);
        ballVelocity = ballVelocity / WHEEL_TO_BALL_VELOCITY_RATIO; // translate required ball velocity to flywheel
                                                                    // velocity

        // LogManager.log("flywheel vel: " + ballVelocity);
        // calculate the hood angle
        hoodAngle = Math.atan(zVel / xyVel) - HOOD_OFFSET; // with hood correction
        // check for max angle
        if (hoodAngle > ShooterConstans.MAX_ANGLE_HOOD) {
          hoodAngle = ShooterConstans.MAX_ANGLE_HOOD;
          ballVelocity = xyVel / Math.cos(hoodAngle);
        }

        // calculate the heading
        Rotation2d ballHeading = new Rotation2d(xVel, yVel);

        // LogManager.log("new hood angle: " + hoodAngle + " ball heading: " +
        // ballHeading);
        RobotCommon.futureAngleFromTargetRobotRelative = MathUtil.angleModulus(ballHeading.getRadians() - nextPose.getRotation().getRadians());
        shooter.setFlywheelVel(ballVelocity);
        shooter.setHoodAngle(hoodAngle);
        shooter.setFeederPower(0.4);

      
        break;

      case SHOOTING:
        robotSpeeds = RobotCommon.fieldRelativeSpeeds;
        nextPose = ShooterUtils.computeFuturePosition(robotSpeeds, RobotCommon.currentRobotPose, 0.1);
        turretPos = nextPose.getTranslation()
            .plus(ShooterConstans.TURRET_POSITION_ON_ROBOT.rotateBy(chassis.getPose().getRotation()));
        toHub = Field.HUB(true).getCenter().getTranslation().minus(turretPos);

        // get the distance, heading and LUT valuse
        distance = toHub.getNorm();
        heading = toHub.getAngle();

        SmartDashboard.putNumber("Shooter distance", distance);
        lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
        vel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO; // correct to actual ball shooting
        hoodAngle = lut[1];

        shooter.setFeederPower(0.4);
        shooter.setVelocitiesAndAngle(lut[0], lut[1]);
        // setShootingAndHood(hoodAngle, vel, heading, robotSpeeds);

        // setShootingAndHood(hoodAngle, vel, heading, RobotCommon.fieldRelativeSpeeds);
        break;

      case TRENCH:
        shooter.setHoodAngle(Math.toRadians(45));
        shooter.setFlywheelVel(10);
        break;

      case TEST:
        shooter.setHoodAngle(Math.toRadians(wantedAngle));
        shooter.setFlywheelVel(wantedVel);
        shooter.setFeederPower(0.4);
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

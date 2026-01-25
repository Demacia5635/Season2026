package frc.robot.Shooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Shooter.utils.ShooterUtils;

public class ShootOnTheFly extends Command {
    private Chassis chassis;
    private Shooter shooter;
    private double HOOD_OFFSET = Math.toRadians(0);
    private double WHEEL_TO_BALL_VELOCITY_RATIO = 0.48;
    private double MAGNUS_CORRECTION = 0.05;
    private double VELOCITY_CORRECTION = 1;

    private boolean shootVelocityWasOK = false;

    public ShootOnTheFly(Chassis chassis, Shooter shooter) {
        this.chassis = chassis;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        ChassisSpeeds robotSpeeds = chassis.getChassisSpeedsFieldRel();
        Pose2d nextPose = ShooterUtils.computeFuturePosition(robotSpeeds, chassis.getPose(), 0.02);
        Translation2d toHub = ShooterConstans.HUB_POSE_Translation2d.minus(nextPose.getTranslation());

        // get the distance, heading and LUT valuse
        double distance = toHub.getNorm();
        Rotation2d heading = toHub.getAngle();

        double[] lut = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance);
        double lutVel = lut[0] * WHEEL_TO_BALL_VELOCITY_RATIO; // correct to actual ball shooting
        double lutHoodAngle = lut[1] + HOOD_OFFSET; // correct to actual ball pitch

        // set the horizontal (xy) velocity and the vertical (z) velocity
        double xyVel = lutVel * Math.cos(lutHoodAngle);
        double zVel = lutVel * Math.sin(lutHoodAngle);

        // calculate the x/y velocities correected by robot speeds
        double xVel = xyVel * heading.getCos() - robotSpeeds.vxMetersPerSecond*VELOCITY_CORRECTION;
        double yVel = xyVel * heading.getSin() - robotSpeeds.vyMetersPerSecond*VELOCITY_CORRECTION;

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
        double hoodAngle = Math.atan(zVel / xyVel) - HOOD_OFFSET; // with hood correction
        // check for max angle
        if(hoodAngle > ShooterConstans.MAX_ANGLE_HOOD) {
            hoodAngle = ShooterConstans.MAX_ANGLE_HOOD;
            ballVelocity = xyVel / Math.cos(hoodAngle);
        }
        
        // calculate the heading
        Rotation2d ballHeading = new Rotation2d(xVel, yVel);

        // LogManager.log("new hood angle: " + hoodAngle + " ball heading: " +
        // ballHeading);
        chassis.setTargetAngle(ballHeading.getRadians());
        shooter.setFlywheelVel(ballVelocity);
        shooter.setHoodAngle(hoodAngle);

        double shooterVel = shooter.getShooterVelocity();
        if (shooterVel > ballVelocity * 0.95) {
            shootVelocityWasOK = true;
        } else if (shootVelocityWasOK && shooterVel < ballVelocity * 0.95) {
            shootVelocityWasOK = false;
            LogManager.log(String.format("=========== Shoot with %3.1f/%3.1f/%3.0f", shooterVel,
                    Math.toDegrees(shooter.getAngleHood()), chassis.getGyroAngle().getDegrees()));
            //new RunCommand(()->chassis.stop(), chassis).schedule();

        }
        if(Math.abs(robotSpeeds.vxMetersPerSecond) > 0.1 || Math.abs(robotSpeeds.vyMetersPerSecond) > 0.1) {
        LogManager.log(String.format(
                "shoot-%3.1f/%3.1f/%3.0f %3.1f/%3.1f shooter %3.1f/%3.1f from-%3.1f/%3.1f/%3.0f to %3.1f/%3.0f/%3.1f/%3.0f - robot vel-%3.1f/%3.1f - %3.1f/%3.1f/%3.1f",
                ballVelocity, Math.toDegrees(hoodAngle), ballHeading.getDegrees(),
                lut[0], Math.toDegrees(lut[1]),
                shooter.getShooterVelocity(), Math.toDegrees(shooter.getAngleHood()),
                nextPose.getTranslation().getX(), nextPose.getTranslation().getY(),
                nextPose.getRotation().getDegrees(),
                toHub.getX(), toHub.getY(), distance, heading.getDegrees(),
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond,
                xVel, yVel, zVel));
        }

    }
}

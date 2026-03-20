// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.utils.sensors.DigitalEncoderConfig;

/** Add your docs here. */

public class ShooterConstans {

        public static final LookUpTable SHOOTER_LOOKUP_TABLE = new LookUpTable(2);
        static {

                SHOOTER_LOOKUP_TABLE.add(1.3, 12.7, Math.toRadians(72));
                SHOOTER_LOOKUP_TABLE.add(2.28, 13.6, Math.toRadians(65));
                SHOOTER_LOOKUP_TABLE.add(3.22, 15, Math.toRadians(62));
                SHOOTER_LOOKUP_TABLE.add(4.55, 16.7, Math.toRadians(58));
                SHOOTER_LOOKUP_TABLE.add(5.53, 18.3, Math.toRadians(54));


                // SHOOTER_LOOKUP_TABLE.add(0.96, 12.7, Math.toRadians(72));
                // SHOOTER_LOOKUP_TABLE.add(2.34, 14, Math.toRadians(68));
                // SHOOTER_LOOKUP_TABLE.add(3.05, 15, Math.toRadians(66));
                // SHOOTER_LOOKUP_TABLE.add(4.39, 17.7, Math.toRadians(60));

                // /* look up table 18.3 */
                // SHOOTER_LOOKUP_TABLE.add(1.3, 12.6, Math.toRadians(73));
                // SHOOTER_LOOKUP_TABLE.add(2.24, 13.5, Math.toRadians(68));
                // SHOOTER_LOOKUP_TABLE.add(3.3, 14.8, Math.toRadians(63));
                // SHOOTER_LOOKUP_TABLE.add(4.41, 17.1, Math.toRadians(61));
                // SHOOTER_LOOKUP_TABLE.add(5.53, 18.3, Math.toRadians(54));

                // SHOOTER_LOOKUP_TABLE.add(1.31, 12.3, Math.toRadians(75));

                // SHOOTER_LOOKUP_TABLE.add(2.38, 13.7, Math.toRadians(68));
                // SHOOTER_LOOKUP_TABLE.add(3.28, 14.7, Math.toRadians(64));
                // SHOOTER_LOOKUP_TABLE.add(4.14, 16.4, Math.toRadians(60));
                // SHOOTER_LOOKUP_TABLE.add(5.23, 17.5, Math.toRadians(55));

        }

        public static final Canbus SHOOTER_CANBUS = Canbus.Rio;

        public static final int SHOOTER_MOTOR_ID = 50;
        public static final String SHOOTER_MOTOR_NAME = "Shooter/Flywheel";

        public static final double SHOOTER_KP = 2d;
        public static final double SHOOTER_KI = 0d;
        public static final double SHOOTER_KS = 0.30414;
        public static final double SHOOTER_KV = 0.47818;
        public static final double SHOOTER_KA = 0.19290;

        public static final double SHHOTER_KV2 = 0.00026;

        public static final TalonFXConfig SHOOTER_MOTOR_CONFIG = new TalonFXConfig(SHOOTER_MOTOR_ID, SHOOTER_CANBUS,
                        SHOOTER_MOTOR_NAME)
                        .withFeedForward(SHHOTER_KV2, 0)
                        .withCurrent(40)
                        .withInvert(false)
                        .withBrake(false)
                        .withPID(SHOOTER_KP, SHOOTER_KI, 0, SHOOTER_KS, SHOOTER_KV, SHOOTER_KA, 0)
                        .withMeterMotor(1, 3 * 0.0254);

        public static final int FEEDER_ID = 52;
        public static final String FEEDER_NAME = "Shooter/Feeder";

        public static final double FEEDER_GEAR_RATIO = 1;

        public static final double FEEDER_POWER = 0.8;

        public static final TalonFXConfig FEEDER_CONFIG = new TalonFXConfig(FEEDER_ID, SHOOTER_CANBUS, FEEDER_NAME)
                        .withRadiansMotor(FEEDER_GEAR_RATIO)
                        .withBrake(false)
                        .withVolts(10)
                        .withInvert(false)
                        .withCurrent(15);

        public static final int HOOD_ID = 51;
        public static final String HOOD_NAME = "Shooter/Hood";

        public static final double HOOD_KP = 24.3;// 0.65;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_KS = 0.08699;
        public static final double HOOD_KV = 1.93076;
        public static final double HOOD_KA = 0.05575;
        public static final double HOOD_KG = 0;
        public static final double HOOD_MAX_VELOCITY = 2.5;
        public static final double HOOD_MAX_ACCEL = Math.PI;
        public static final double HOOD_MAX_JERK = 0;
        public static final double HOOD_GEAR_RATIO = 2 * 64;

        public static final double MAX_HOOD_ANGLE_ERROR = Math.toRadians(0.8); // radians

        public static final TalonFXConfig HOOD_CONFIG = new TalonFXConfig(HOOD_ID, SHOOTER_CANBUS, HOOD_NAME)
                        .withBrake(true)
                        .withRadiansMotor(HOOD_GEAR_RATIO)
                        .withCurrent(20)
                        .withInvert(true)
                        .withMaxPositionError(MAX_HOOD_ANGLE_ERROR)
                        .withPID(HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KS, HOOD_KV, HOOD_KA, HOOD_KG)
                        .withMotionParam(HOOD_MAX_VELOCITY, HOOD_MAX_ACCEL, HOOD_MAX_JERK);

        public static final double MAX_ANGLE_HOOD = Math.toRadians(83d);
        public static final double MIN_ANGLE_HOOD = Math.toRadians(47d);

        public static final int HOOD_ENCODER_CHANNEL = 7;
        public static final String HOOD_ENCODER_NAME = "Shooter/Hood/Angle Encoder";
        public static final double HOOD_OFFSET = Math.toRadians(86) - 0.1161603883664826; // 0.752882679432794;
        public static final DigitalEncoderConfig HOOD_ENCODER_CONFIG = new DigitalEncoderConfig(HOOD_ENCODER_CHANNEL,
                        HOOD_ENCODER_NAME)
                        .withInvert(false);

        public static final Translation3d HUB_POSE_Translation3d = new Translation3d(11.265 + 0.5969, 4.023, 1.829);
        public static final Translation2d HUB_POSE_Translation2d = HUB_POSE_Translation3d.toTranslation2d();
        public static final Pose2d hubPose2d = new Pose2d(HUB_POSE_Translation2d, new Rotation2d());

        public static final Translation2d DELIVERY_POINT1 = new Translation2d(12.8619, 1.5023);
        public static final Translation2d DELIVERY_POINT2 = new Translation2d(12.8619, 6.523);

        public static final Translation2d TURRET_POSITION_ON_ROBOT = new Translation2d(-0.113, 0.190);
        public static final double MAX_FLYWHEEL_VELOCITY_ERROR = 0.3; // m/s

}

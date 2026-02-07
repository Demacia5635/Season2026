package frc.robot.climb.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.DigitalEncoderConfig;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

public class ClimbConstants {
        public static final int MOTOR_ID_ARMS = 51;
        public static final Canbus CANBUS = Canbus.Rio;
        public static final boolean WITH_BRAKE_ARMS = true;
        public static final double MAX_CURRENT = 40;
        public static final double DIAMETER_ARMS = 0;
        public static final double GEAR_RATIO_ARMS = 1 / 24;
        public static final boolean WITH_INVERT_ARMS = false;
        public static final double POWER_TO_LOWER_ARMS = 0.2;
        public static final double POWER_TO_RAISE_ARMS = 0.2;
        public static final double TIME_TO_RAISE_ARMS = 0.2;
        public static final double TIME_TO_RAISE_ARMS_AFTER_CLIMB = 0;
        public static final double ANGLE_ARMS_RAISED = 0.6399424235362409; // radians
        public static final double ANGLE_ARMS_LOWERED = Math.toRadians(0);
        public static final double ARMS_ANGLE_CLOSED = 3.901700996125844; // radians
        public static final double armsOffset = 0;
        public static final double CHASSIS_TOLERANCE = 0.05; // meters
        public static final double rotationKp = 2.2;
        public static final double driveKp = 1.3;
        public static Pose2d targetRightSide = Pose2d.kZero;
        public static Pose2d targetLeftSide = Pose2d.kZero;
        public static Pose2d targetToFullyCloseArms = Pose2d.kZero;
        public static Pose2d targetToOpenArmsAfterClimb = Pose2d.kZero;

        public static final int MOTOR_ID_LEVER = 52;
        public static final boolean WITH_BRAKE_LEVER = false;
        public static final double DIAMETER_LEVER = 0;
        public static final double LEVER_GEAR_RATIO = 1 / 64;
        public static final boolean WITH_INVERT_LEVER = false;
        public static final double POWER_TO_OPEN_LEVER = 0.2;
        public static final double POWER_TO_CLOSE_LEVER = -0.2;
        public static final double ANGLE_LEVER_CLOSE = -0.4393564709457465; // radians
        public static final double ANGLE_LEVER_OPEN = 0.2854944237345549; // radians
        public static final double CLOSE_LEVER_TOLERANCE = Math.toRadians(5); // radians
        public static final double POWER_TO_KEEP_HEIGHT = 0.05;

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KG = 0;
        public static final double KD = 0;
        public static final double KI = 0;

        public static final TalonSRXConfig ARMS_MOTOR_CONFIG = new TalonSRXConfig(MOTOR_ID_ARMS, "arms motor")
                        .withBrake(WITH_BRAKE_ARMS)
                        .withCurrent(MAX_CURRENT)
                        .withMeterMotor(GEAR_RATIO_ARMS, DIAMETER_ARMS)
                        .withInvert(WITH_INVERT_ARMS);

        public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS, "motor lever")
                        .withBrake(WITH_BRAKE_LEVER)
                        .withCurrent(MAX_CURRENT)
                        .withInvert(WITH_INVERT_LEVER)
                        .withRadiansMotor(LEVER_GEAR_RATIO);

        public static final DigitalEncoderConfig DIGITAL_ENCODER_CONFIG = new DigitalEncoderConfig(9, "climb encoder")
                        .withInvert(false)
                        .withOffset(armsOffset);

        public enum CLIMB_STATE {
                IDLE,
                CLOSE,
                PREP_CLIMB,
                CLIMB,
                GET_OFF_CLIMB,
                TESTING;
        }
}

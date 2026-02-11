package frc.robot.climb.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.DigitalEncoderConfig;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

public class ClimbConstants {
        public static final Canbus CANBUS_CLIME = Canbus.Rio;

        //arm constans
        public static final boolean WITH_BRAKE_ARMS = true;
        public static final double MAX_CURRENT = 40;
        public static final int MOTOR_ID_ARMS = 51;
        public static final double DIAMETER_ARMS = 0;
        public static final double GEAR_RATIO_ARMS = 1 / 24;
        public static final boolean WITH_INVERT_ARMS = false;        
        public static final double ANGLE_ARMS_RAISED = 0.6399424235362409; // radians
        public static final double ANGLE_ARMS_LOWERED = Math.toRadians(0);
        public static final double ARMS_ANGLE_CLOSED = 3.901700996125844; // radians
        public static final double armsOffset = 0;

        //arm pid feet forward
        public static final double ARMS_KS = 0;
        public static final double ARMS_KV = 0;
        public static final double ARMS_KA = 0;
        public static final double ARMS_KP = 0;
        public static final double ARM_KG = 0;
        public static final double ARMS_KD = 0;
        public static final double ARMS_KI = 0;

        
        //clime armd motor
        public static final TalonSRXConfig ARMS_MOTOR_CONFIG = new TalonSRXConfig(MOTOR_ID_ARMS, "arms motor")
                        .withPID(ARMS_KP, ARMS_KI, ARMS_KD, ARMS_KS, ARMS_KV, ARMS_KA, ARM_KG)
                        .withBrake(WITH_BRAKE_ARMS)
                        .withCurrent(MAX_CURRENT)
                        .withMeterMotor(GEAR_RATIO_ARMS, DIAMETER_ARMS)
                        .withInvert(WITH_INVERT_ARMS);


        //CLIME LEVER MOTOR POD AND FEET FORWARD
        public static final double LEVER_KS = 0;
        public static final double LEVER_KV = 0;
        public static final double LEVER_KA = 0;
        public static final double LEVER_KP = 0;
        public static final double LEVER_KG = 0;
        public static final double LEVER_KD = 0;
        public static final double LEVER_KI = 0;


        //lever constans
        public static final int MOTOR_ID_LEVER = 52;
        public static final boolean WITH_BRAKE_LEVER = false;
        public static final double DIAMETER_LEVER = 0;
        public static final double LEVER_GEAR_RATIO = 1 / 64;
        public static final boolean WITH_INVERT_LEVER = false;
        public static final double ANGLE_LEVER_CLOSED = -0.4393564709457465; // radians
        public static final double ANGLE_LEVER_OPEN = 0.2854944237345549; // radians
        public static final double CLOSE_LEVER_TOLERANCE = Math.toRadians(5); // radians

        //lever clime motor config
        public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS_CLIME, "motor lever")
                        .withPID(ARMS_KP, ARMS_KI, ARMS_KD, ARMS_KS, ARMS_KV, ARMS_KA, ARM_KG)
                        .withBrake(WITH_BRAKE_LEVER)
                        .withCurrent(MAX_CURRENT)
                        .withInvert(WITH_INVERT_LEVER)
                        .withRadiansMotor(LEVER_GEAR_RATIO);

        //the clime arm encodr
        public static final DigitalEncoderConfig DIGITAL_ENCODER_CONFIG = new DigitalEncoderConfig(9, "climb encoder")
                        .withInvert(false)
                        .withOffset(armsOffset);


        //constans to auto drive to tower
        public static final double CHASSIS_TOLERANCE = 0.05; // meters
        public static final double rotationKp = 2.2;
        public static final double driveKp = 1.3;

        //TODO: toke with tomer move it to the fild class
        //clime pose
        public static Pose2d targetRightSide = Pose2d.kZero;
        public static Pose2d targetLeftSide = Pose2d.kZero;
        public static Pose2d targetToFullyCloseArms = Pose2d.kZero;
        public static Pose2d targetToOpenArmsAfterClimb = Pose2d.kZero;
}

package frc.robot.climb.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.DigitalEncoderConfig;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

public class ClimbConstants {
        public static final Canbus CANBUS_CLIMB = Canbus.Rio;

        //arm constans
        public static final boolean WITH_BRAKE_ARMS = true;
<<<<<<< HEAD
        public static final double MAX_CURRENT = 20;
        public static final int MOTOR_ID_ARMS = 61;//TODO: CAMGE IN TUNER
        public static final double DIAMETER_ARMS = 0;
        public static final double GEAR_RATIO_ARMS = 24;
        public static final boolean WITH_INVERT_ARMS = false;        
        public static final double ANGLE_ARMS_RAISED = 0.6399424235362409; // radians
        public static final double ANGLE_ARMS_LOWERED = Math.toRadians(22.86102);
=======
        public static final double MAX_CURRENT = 40;
        public static final int MOTOR_ID_ARMS = 61;
        public static final double GEAR_RATIO_ARMS = 24;
        public static final boolean WITH_INVERT_ARMS = false;        
        public static final double ANGLE_ARMS_RAISED = 0.6399424235362409; // radians
        public static final double ANGLE_ARMS_LOWERED =0 ;
>>>>>>> ae129d028332b347470fc5c5b7f653677da87846
        public static final double ARMS_ANGLE_CLOSED = 3.901700996125844; // radians
        public static final double ARMS_OFFSET = 1.6465087097464108;

        public static final double ARMS_KP = 3;

        
        //climb arms motor
        public static final TalonSRXConfig ARMS_MOTOR_CONFIG = new TalonSRXConfig(MOTOR_ID_ARMS, "arms motor")
                        .withBrake(WITH_BRAKE_ARMS)
                        .withCurrent(MAX_CURRENT)
<<<<<<< HEAD
=======
                        .withRadiansMotor(GEAR_RATIO_ARMS)
>>>>>>> ae129d028332b347470fc5c5b7f653677da87846
                        .withInvert(WITH_INVERT_ARMS);


        //CLIMB LEVER MOTOR PID AND FEEDFORWARD
        public static final double LEVER_KS = 0;
        public static final double LEVER_KV = 0;
        public static final double LEVER_KA = 0;
        public static final double LEVER_KP = 0;
        public static final double LEVER_KG = 0;
        public static final double LEVER_KD = 0;
        public static final double LEVER_KI = 0;


        //lever constans
        public static final int MOTOR_ID_LEVER = 60;
<<<<<<< HEAD
        public static final boolean WITH_BRAKE_LEVER ;
        public static final double DIAMETER_LEVER = 0;
=======
        public static final boolean WITH_BRAKE_LEVER = false;
>>>>>>> ae129d028332b347470fc5c5b7f653677da87846
        public static final double LEVER_GEAR_RATIO = 64;
        public static final boolean WITH_INVERT_LEVER = false;
        public static final double ANGLE_LEVER_CLOSED = -0.4393564709457465; // radians
        public static final double ANGLE_LEVER_OPEN = 0.2854944237345549; // radians

        //lever climb motor config
        public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS_CLIMB, "motor lever")
                        .withPID(LEVER_KP, LEVER_KI, LEVER_KD, LEVER_KS, LEVER_KV, LEVER_KA, LEVER_KG)
                        .withBrake(WITH_BRAKE_LEVER)
                        .withCurrent(MAX_CURRENT)
                        .withInvert(WITH_INVERT_LEVER)
                        .withRadiansMotor(LEVER_GEAR_RATIO);

        //the climb arm encoder
        public static final DigitalEncoderConfig DIGITAL_ENCODER_CONFIG = new DigitalEncoderConfig(3, "climb encoder")
                        .withInvert(true);


        //constans to auto drive to tower
        public static final double rotationKp = 2.2;
        public static final double driveKp = 1.3;
        public static final double velocityToStraightenArms = 0.5; 
        public static final double velocityToRaiseArmsAfterClimb = 0.5; 
        public static final double CHASSIS_TOLERANCE = 0; // radians
        public static final double timeToStraightenArms = 0.2; // seconds
        public static final double timeToRaiseArmsAfterClimb = 0.1; // seconds



        //climb pose
        public static Pose2d targetRightSide = Pose2d.kZero;
        public static Pose2d targetLeftSide = Pose2d.kZero;
      
}

package frc.robot.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {
    public static final String NAME = "Turret";

    public static final String MOTOR_NAME = "Turret Motor";

    public static final double MAX_TURRET_ANGLE = 1.94;
    public static final double MIN_TURRET_ANGLE = -2.019531;

    private static final int TURRET_MOTOR_ID = 40;
    private static final Canbus CANBUS = Canbus.Rio;
    private static final double GEAR_RATIO = ((48d*112d)/27d);
    
    private static final double KP = 24;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KS = 0.05979;
    private static final double KV = 3.4;
    private static final double KA = 0.248;
    
    private static final double MAX_VELOCITY = Math.toRadians(160);
    private static final double MAX_ACCEL = Math.toRadians(300);
    private static final double MAX_JERK = Math.toRadians(2000);    
    
    public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_ID, CANBUS, MOTOR_NAME)
    .withBrake(true)
    .withInvert(false)
    .withRadiansMotor(GEAR_RATIO)
    .withPID(KP, KI, KD, KS, KV, KA, 0)
    .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK);

    
    public static final String LIMIT_SWITCH_MIN_NAME = "Min Limit Switch";
    public static final LimitSwitchConfig LIMIT_SWITCH_MIN_CONFIG = new LimitSwitchConfig(1, LIMIT_SWITCH_MIN_NAME); 
    public static final String LIMIT_SWITCH_MAX_NAME = "Max Limit Switch";
    public static final LimitSwitchConfig LIMIT_SWITCH_MAX_CONFIG = new LimitSwitchConfig(0, LIMIT_SWITCH_MAX_NAME); 

    public static final Translation2d TURRET_POS = Translation2d.kZero;

}

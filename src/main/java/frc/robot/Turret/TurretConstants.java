package frc.robot.Turret;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {

    public static final double MAX_TURRET_ANGLE = 171;
    public static final double MIN_TURRET_ANGLE = -171;

    private static final int TURRET_MOTOR_ID = 10;
    private static final Canbus CANBUS = Canbus.Rio;
    private static final double GEAR_RATIO = 1/2d;
    
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 0;
    private static final double kV = 0;
    private static final double kA = 0;
    
    private static final double MAX_VELOCITY = Math.toRadians(180);
    private static final double MAX_ACCEL = Math.toRadians(360);
    private static final double MAX_JERK = Math.toRadians(720);    
    
    public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_ID, CANBUS, "Turret Motor")
    .withBrake(true)
    .withRadiansMotor(GEAR_RATIO)
    .withPID(kP, kI, kD, kS, kV, kA, 0)
    .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK);

    public static final LimitSwitchConfig LIMIT_SWITCH_MIN_CONFIG = new LimitSwitchConfig(9, "Min Limit Switch"); 
    public static final LimitSwitchConfig LIMIT_SWITCH_MAX_CONFIG = new LimitSwitchConfig(8, "Max Limit Switch"); 

}

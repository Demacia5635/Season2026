package frc.robot.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {

    public static final double MAX_TURRET_ANGLE = Math.toRadians(110);
    public static final double MIN_TURRET_ANGLE = -Math.toRadians(110);

    private static final int TURRET_MOTOR_ID = 40;
    private static final Canbus CANBUS = Canbus.Rio;
    private static final double GEAR_RATIO = ((64d*112d)/27d);
    
    private static final double kP = 1;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 0.02501;
    private static final double kV = 4.85663;
    private static final double kA = 0.00005;
    
    private static final double MAX_VELOCITY = Math.toRadians(120);
    private static final double MAX_ACCEL = Math.toRadians(180);
    private static final double MAX_JERK = Math.toRadians(2000);    
    
    public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_ID, CANBUS, "Turret/Motor")
    .withBrake(true)
    .withInvert(false)
    .withRadiansMotor(GEAR_RATIO)
    .withPID(kP, kI, kD, kS, kV, kA, 0)
    .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK);

    public static final LimitSwitchConfig LIMIT_SWITCH_MIN_CONFIG = new LimitSwitchConfig(1, "Min Limit Switch"); 
    public static final LimitSwitchConfig LIMIT_SWITCH_MAX_CONFIG = new LimitSwitchConfig(0, "Max Limit Switch"); 

    public static final Translation2d TURRET_POS = Translation2d.kZero;

}

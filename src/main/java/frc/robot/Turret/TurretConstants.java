package frc.robot.Turret;

import edu.wpi.first.math.geometry.Translation2d;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {

    public static final double MAX_TURRET_ANGLE = 360;
    public static final double MIN_TURRET_ANGLE = Math.toRadians(30);

    public static final double MIN_SENSOR = 2 * Math.PI - 0.3354;
    public static final double MAX_SENSOR = 0;

    private static final int TURRET_MOTOR_ID = 40;
    private static final Canbus CANBUS = Canbus.CANIvore;
    private static final double GEAR_RATIO = ((36d * 112d) / 27d);

    private static final double kP = 0.8;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 0.03248;
    private static final double kV = 2.75969;
    private static final double kA = 0.13299;

    private static final double MAX_VELOCITY = Math.toRadians(210);
    private static final double MAX_ACCEL = Math.toRadians(500);
    private static final double MAX_JERK = Math.toRadians(0);

    public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_ID, CANBUS, "Turret/Motor")
            .withBrake(true)
            .withInvert(false)
            .withRadiansMotor(GEAR_RATIO)
            .withPID(kP, kI, kD, kS, kV, kA, 0)
            .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK);

    public static final LimitSwitchConfig LIMIT_SWITCH_MAX_CONFIG = new LimitSwitchConfig(0, "Max Limit Switch");
    public static final LimitSwitchConfig LIMIT_SWITCH_MIN_CONFIG = new LimitSwitchConfig(7, "Min Limit Switch");

    public static final double MAX_ALLOWED_ANGLE_ERROR = Math.toRadians(1.2);

    public static final Translation2d TURRET_POSITION_ON_ROBOT = new Translation2d(-0.113, 0.190);
}

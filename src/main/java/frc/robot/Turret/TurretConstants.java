package frc.robot.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {

    public static final double MAX_TURRET_ANGLE = 360; 
    public static final double MIN_TURRET_ANGLE = Math.toRadians(30);

    public static final double MIN_SENSOR = 2 * Math.PI - 0.3354;

    private static final int TURRET_MOTOR_ID = 40;
    private static final Canbus CANBUS = Canbus.CANIvore;
    private static final double GEAR_RATIO = ((36d * 112d) / 27d);

    private static final double kP = 19.2d;//0.5;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 0.03737;
    private static final double kV = 2.75556;
    private static final double kA = 0.12839;

    private static final double MAX_VELOCITY = Math.toRadians(210);
    private static final double MAX_ACCEL = Math.toRadians(300);
    private static final double MAX_JERK = Math.toRadians(0);

    public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_ID, CANBUS, "Turret/Motor")
            .withBrake(true)
            .withInvert(false)
            .withRadiansMotor(GEAR_RATIO)

            .withPID(kP, kI, kD, kS, kV, kA, 0)
            .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK);
    public static final LimitSwitchConfig LIMIT_SWITCH_MAX_CONFIG = new LimitSwitchConfig(9, "Max Limit Switch");
    public static final LimitSwitchConfig LIMIT_SWITCH_MIN_CONFIG = new LimitSwitchConfig(7, "Min Limit Switch");

    public static final double MAX_ALLOWED_ANGLE_ERROR = Math.toRadians(0.8);

    public static final double CURRENT_FOR_LIMIT = 30; //need to test
    public static final double TIME_UNDER_CURRENT_FOR_LIMIT = 0.3; //seconds, need to test

}

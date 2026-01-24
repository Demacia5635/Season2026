package frc.robot.climb.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

public class ClimbConstants {
    public static final int MOTOR_ID_ARMS = 1;
    public static final Canbus CANBUS = Canbus.Rio;
    public static final boolean WITH_BRAKE_ARMS = true;
    public static final double MAX_CURRENT = 40;
    public static final double DIAMETER_ARMS = 0;
    public static final double GEAR_RATIO_ARMS = 0;
    public static final boolean WITH_INVERT_ARMS = false;
    public static final double ANGLE_ARMS_OPEN = 70; //degrees
    public static final double POWER_TO_LOWER_ARMS = 0.2;
    public static final TalonFXConfig ARMS_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_ARMS, CANBUS, "motor climb")
            .withBrake(WITH_BRAKE_ARMS)
            .withCurrent(MAX_CURRENT)
            .withMeterMotor(GEAR_RATIO_ARMS, DIAMETER_ARMS)
            .withInvert(WITH_INVERT_ARMS);

    public static final int MOTOR_ID_LEVER = 2;
    public static final boolean WITH_BRAKE_LEVER = false;
    public static final double DIAMETER_LEVER = 0;
    public static final double LEVER_GEAR_RATIO = 0;
    public static final boolean WITH_INVERT_LEVER = false;
    public static final double POWER_TO_RAISE_LEVER = -0.2;
    public static final double ANGLE_LEVER_OPEN = 60; //degrees



    public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS, "motor climb")
            .withBrake(WITH_BRAKE_LEVER)
            .withCurrent(MAX_CURRENT)
            .withMeterMotor(LEVER_GEAR_RATIO, DIAMETER_LEVER)
            .withInvert(WITH_INVERT_LEVER);

}

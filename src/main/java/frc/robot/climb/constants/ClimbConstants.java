package frc.robot.climb.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;
import frc.demacia.utils.motors.TalonFXConfig;

public class ClimbConstants {
public static final int MOTOR_ID = 1;
public static final Canbus CANBUS = Canbus.Rio;
public static final boolean WITH_BRAKE= true;
public static final double MAX_CURRENT = 40;
public static final double DIAMETER = 0;
public static final double GEAR_RATIO = 0;
public static final boolean WITH_INVERT = false;
public static final int LIMIT_SWITCH_ID = 3;

public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID, CANBUS,"motor climb")
.withBrake(WITH_BRAKE)
.withCurrent(MAX_CURRENT)
.withMeterMotor(GEAR_RATIO,DIAMETER)
.withInvert(WITH_INVERT);

public static final LimitSwitchConfig lIMIT_Config = new LimitSwitchConfig(LIMIT_SWITCH_ID, "limit switch climb");
}

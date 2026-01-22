package frc.robot.climb.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

public class ClimbConstants {
    public static final int MOTOR_ID = 15;  
    public static final String MOTOR_NAME = "Climb Motor";
    public static final Canbus CANBUS = Canbus.Rio;
    public static final boolean IS_BRAKE = true;
    public static final boolean WITH_INVERT = false;
    public static final double MAX_CURRENT = 40.0; 



    public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID, CANBUS, MOTOR_NAME)
            .withBrake(IS_BRAKE)
            .withInvert(WITH_INVERT)
            .withCurrent(MAX_CURRENT);
}
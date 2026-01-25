package frc.robot.climb.constants;

import edu.wpi.first.units.measure.Time;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

public class ClimbConstants {
    public static final int MOTOR_ID_ARMS = 1;
    public static final Canbus CANBUS = Canbus.Rio;
    public static final boolean WITH_BRAKE_ARMS = true;
    public static final double MAX_CURRENT = 40;
    public static final double DIAMETER_ARMS = 0;
    public static final double GEAR_RATIO_ARMS = 1/24;
    public static final boolean WITH_INVERT_ARMS = false;
    public static final double POWER_TO_LOWER_ARMS = 0.2;
    public static final double POWER_TO_RAISE_ARMS = 0.2;
    public static final double TIME_TO_RAISE_ARMS = 0.2;
    public static final double TIME_TO_RAISE_ARMS_AFTER_CLIMB = 0;



    public static final TalonSRXConfig ARMS_MOTOR_CONFIG = new TalonSRXConfig(MOTOR_ID_ARMS, "arms motor")

            .withBrake(WITH_BRAKE_ARMS)
            .withCurrent(MAX_CURRENT)
            .withMeterMotor(GEAR_RATIO_ARMS, DIAMETER_ARMS)
            .withInvert(WITH_INVERT_ARMS);

    public static final double CLOSE_LEVER_TOLERANCE = Math.toRadians(5);//radians
    public static final int MOTOR_ID_LEVER = 2;
    public static final boolean WITH_BRAKE_LEVER = false;
    public static final double DIAMETER_LEVER = 0;
    public static final double LEVER_GEAR_RATIO = 1/64;
    public static final boolean WITH_INVERT_LEVER = false;
    public static final double POWER_TO_OPEN_LEVER = 0.2;
    public static final double POWER_TO_CLOSE_LEVER = -0.2;
    public static final double ANGLE_LEVER_CLOSE = Math.toRadians(0); //radians
    public static final double ANGLE_LEVER_OPEN = Math.toRadians(0);//``radians
    public static final double POWER_TO_KEEP_HEIGHT = 0.05;


    public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS, "motor lever")
            .withBrake(WITH_BRAKE_LEVER)
            .withCurrent(MAX_CURRENT)
            .withMeterMotor(LEVER_GEAR_RATIO, DIAMETER_LEVER)
            .withInvert(WITH_INVERT_LEVER);

}

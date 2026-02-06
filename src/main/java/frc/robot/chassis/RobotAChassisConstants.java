package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.Mk5nConstants;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;
import frc.demacia.vision.TagPose;

public final class RobotAChassisConstants {
    public static final String NAME = "Robot A Chassis";
    public static final Canbus CANBUS = Canbus.Rio;

    private static final class DriveMotor {
        private static final String NAME = "Drive Motor";

        private static final double GEAR_RATIO = Mk5nConstants.R2.driveGearRatio;
        private static final double WHEEL_DIAMETER = Mk5nConstants.WHEEL_DIAMETER;
        private static final boolean IS_BRAKE = true;
        private static final boolean IS_INVERT = true;

        private static final double KP = 0d;
        private static final double KI = 0d;
        private static final double KD = 0d;
        private static final double KS = 0d;
        private static final double KV = 0d;
        private static final double KA = 0d;
        private static final double KG = 0d;

        private static TalonFXConfig getDriveMotor(int id, String moduleName) {
            return new TalonFXConfig(id, CANBUS, RobotAChassisConstants.NAME + "/" + moduleName + "/" + NAME)
                    .withBrake(IS_BRAKE)
                    .withInvert(IS_INVERT)
                    .withPID(KP, KI, KD, KS, KV, KA, KG)
                    .withMeterMotor(GEAR_RATIO, WHEEL_DIAMETER);
        }
    }

    private static final class SteerMotor {
        private static final String NAME = "Steer Motor";

        private static final double RAMP_TIME = 0.25;
        private static final double GEAR_RATIO = Mk5nConstants.STEER_GEAR_RATIO;
        private static final boolean IS_BRAKE = true;
        private static final boolean IS_INVERT = true;

        private static final double KP = 0d;
        private static final double KI = 0d;
        private static final double KD = 0d;
        private static final double KS = 0d;
        private static final double KV = 0d;
        private static final double KA = 0d;
        private static final double KG = 0d;

        private static TalonFXConfig getSteerMotor(int id, String moduleName) {
            return new TalonFXConfig(id, CANBUS, RobotAChassisConstants.NAME + "/" + moduleName + "/" + NAME)
                    .withBrake(IS_BRAKE)
                    .withInvert(IS_INVERT)
                    .withRampTime(RAMP_TIME)
                    .withPID(KP, KI, KD, KS, KV, KA, KG)
                    .withRadiansMotor(GEAR_RATIO);
        }
    }

    private static final int PIGEON_ID = 14;
    private static final PigeonConfig pigeonConfig = new PigeonConfig(PIGEON_ID, CANBUS, NAME + "/" + "Pigeon");

    private static SwerveModuleConfig[] getSwerveModuleConfigs(double[] offsets) {
        SwerveModuleConfig[] ans = new SwerveModuleConfig[4];
        for (int i = 0; i < 4; i++) {
            String moduleName = "Error";
            switch (i) {
                case 0:
                    moduleName = "Front Left";
                    break;

                case 1:
                    moduleName = "Front Right";
                    break;

                case 2:
                    moduleName = "Back Left";
                    break;

                case 3:
                    moduleName = "Back Right";
                    break;

                default:
                    break;
            }

            ans[i] = new SwerveModuleConfig(
                    moduleName,
                    SteerMotor.getSteerMotor(i * 3 + 2, moduleName),
                    DriveMotor.getDriveMotor(i * 3 + 1, moduleName),
                    new CancoderConfig(i * 3 + 3, CANBUS, NAME + "/" + moduleName + "/" + "Cancoder"))
                    .withPosion(new Translation2d(
                            i == 0 || i == 1 ? 0.295 : -0.295,
                            i == 0 || i == 2 ? 0.395 : -0.395))
                    .withSteerOffset(offsets[i]);
        }

        return ans;
    }

    private static final SwerveModuleConfig[] modules = getSwerveModuleConfigs(
            new double[] {
                    /* Front Left Offset: */ 0d,
                    /* Front Right Offset: */ 0d,
                    /* Back Left Offset: */ 0d,
                    /* Back Right Offset: */ 0
            });

    public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(NAME, modules, pigeonConfig, new TagPose[] {});
}

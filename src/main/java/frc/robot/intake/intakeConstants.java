// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

/** Add your docs here. */
public class IntakeConstants {
    public static final TalonFXConfig INTAKE_CONFIG = new TalonFXConfig(-1, Canbus.Rio, "intake motor");

    public static final double MAX_POWER = 0.8;

    public static final TalonFXConfig INDEXER_ON_TOP_CONFIG = new TalonFXConfig(21, Canbus.Rio, "Indexer on top motor").withBrake(true).withInvert(true);
    public static final TalonSRXConfig INDEXER_CLOSE_CONFIG = new TalonSRXConfig(22, "Indexer close motor").withBrake(true);
    public static final TalonSRXConfig INDEXER_FAR_CONFIG = new TalonSRXConfig(23, "Indexer far motor").withBrake(true).withInvert(true);
    public static final TalonFXConfig BATTERY_CONFIG = new TalonFXConfig(24, Canbus.Rio, "Battery Motor").withBrake(true).withInvert(false).withPID(20, 0, 0, 0, 0, 0, 0).withRadiansMotor(36);
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

/** Add your docs here. */
public class intakeConstants {

    public static final int ID = 20;
    public static final TalonFXConfig INTAKE_CONFIG = new TalonFXConfig(ID, Canbus.Rio, "intake motor");

    public static final double MAX_POWER = 0.8;

    public static final int SHINA_ID = 34;

    // public static final TalonSRXConfig ROLLER_CONFIG = new TalonSRXConfig(SHINA_ID, "rooler");

    // public static final TalonSRXConfig TO_SHOOTER_CONFIG = new TalonSRXConfig(13, "shit");
}

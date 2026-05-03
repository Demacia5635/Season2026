package frc.robot;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class motorConstants {
public static final int frontLeftMotor = 3;
public static final String frontLeftMotorName = "frontLeftMotor";
public static final Canbus CANBUS = Canbus.Rio;
public static final double maxCurrent = 40.0;
public static final double maxVelocity = 5000.0;
public static final double sec = 1.0;



public static final TalonFXConfig frontLeftMotorConfig = new TalonFXConfig(frontLeftMotor,CANBUS,frontLeftMotorName)
.detectStallInMotor(maxCurrent, maxVelocity, sec, frontLeftMotorName -> System.out.println(frontLeftMotorName + " is stalled! Current: " )

);
   
}

package frc.robot;


import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class motorConstants {
public static final int MotorTestingId = 3;
public static final String MotorTestingName = "Motor Testing";
public static final Canbus CANBUS = Canbus.Rio;
public static final double maxCurrent = 5;
public static final double maxVelocity = 3;
public static final double sec = 0.3;

public static final TalonFXConfig MotorTesting = new TalonFXConfig(MotorTestingId,CANBUS,MotorTestingName)
.withDetectStallInMotor(maxCurrent, maxVelocity, sec, print -> {
    System.out.println("Motor Stalled!");
  
}

);

   
}

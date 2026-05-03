package frc.robot;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class motorConstants {
public static MotorTesting motorTesting;
public static final int MotorTestingId = 3;
public static final String MotorTesting= "Motor Testing";
public static final Canbus CANBUS = Canbus.Rio;
public static final double maxCurrent = 2;
public static final double maxVelocity = 3;
public static final double sec = 0.3;

public static final TalonFXConfig frontLeftMotorConfig = new TalonFXConfig(MotorTestingId,CANBUS,MotorTesting)
.detectStallInMotor(maxCurrent, maxVelocity, sec, talon -> {
  // Handle stall detection gracefully - stop the motor and log the event
  try {
    // Set duty cycle to 0 to ensure motor stops immediately
    talon.setDuty(0);
    talon.stop();
    frc.demacia.utils.log.LogManager.log("Motor stall detected on " + talon.getName() + " - motor stopped immediately");
  } catch (Exception e) {
    // Catch any exceptions that mig7ht occur during stall handling
    frc.demacia.utils.log.LogManager.log("Error handling motor stall: " + e.getMessage());
  }
});

   
}

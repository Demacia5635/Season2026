// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.climb.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.climb.constants.ClimbConstants;
// import frc.robot.climb.subsystems.Climb;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CalibrateLever extends Command {
//   Climb climb;
//   private double currentSpikeCounter = 0;
//   private static final double CURRENT_THRESHOLD = 0; 
//   private final int CLOSE_LEVER_CYCLE_TO_STOP = 5;
//   private boolean IS_LEVER_CLOSED = false;
//   /** Creates a new CalibrateLever. */
//   public CalibrateLever(Climb climb) {
//     this.climb = climb;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     IS_LEVER_CLOSED = false;
//     currentSpikeCounter = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   double currentAmper = climb.getCurrentAmpersLever();
//     if (currentAmper >= CURRENT_THRESHOLD) {
//       currentSpikeCounter++;
//     } else {
//       currentSpikeCounter = 0;
//     }
//     if (currentSpikeCounter >= CLOSE_LEVER_CYCLE_TO_STOP) {
//       climb.stopLever();
//       IS_LEVER_CLOSED = true;
//     } else {
//       climb.setLeverDuty(ClimbConstants.POWER_TO_CLOSE_LEVER);
//     }
//   }
  

  
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     climb.stopLever();
//     climb.resetLeverEncoder();
//     }
  

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return IS_LEVER_CLOSED;
//   }
// }


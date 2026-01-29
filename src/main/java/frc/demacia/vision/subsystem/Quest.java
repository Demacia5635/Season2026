package frc.demacia.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.sensors.Pigeon;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.demacia.vision.utils.VisionConstants.*;

import java.util.function.Supplier;

public class Quest extends SubsystemBase {
  private Field2d field;
  private Field2d field2;
  private Field2d field3;
  private Field2d field4;
  private Field2d field5;
  private QuestNav questNav;
  private Pose3d currentQuestPose;
  private double timestamp;
  private Supplier<Rotation2d> gyroAngle;

    private Pose3d qPose3d = Pose3d.kZero;
  
  public Quest(Supplier<Rotation2d> gyroAngle) {
    timestamp = 0;
    questNav = new QuestNav();
    questNav.commandPeriodic();
    this.gyroAngle = gyroAngle;

    field = new Field2d();
    field2 = new Field2d();
    field3 = new Field2d();
    field4 = new Field2d();
    field5 = new Field2d();
    currentQuestPose = new Pose3d(); // Initialize to origin - IMPORTANT!

    addLog();
  }
  
  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager.addEntry("Quest/Latency", questNav::getLatency).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);
    LogManager.addEntry("Quest/Battery", questNav::getBatteryPercent).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);
    LogManager.addEntry("Quest/LibVersion", questNav::getLibVersion).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);

    // SmartDashboard.putData("Quest/Field", field);
    SmartDashboard.putData("Quest/Field22", field2);
    SmartDashboard.putData("Quest/Field3", field3);
    SmartDashboard.putData("Quest/Field4", field4);
    SmartDashboard.putData("Quest/Field5", field5);
  }

  // Set robot pose (transforms to Quest frame and sends to QuestNav)
  public void setQuestPose(Pose3d currentBotpose) {
    // currentQuestPose = currentBotpose.transformBy(ROBOT_TO_QUEST);

    questNav.setPose(currentBotpose.transformBy(ROBOT_TO_QUEST3D));// the transformBy is to switch x & y and gives back
                                                                   // the hight of the quest

  }

  /**
   * * @return the center of the robot form quest
   */
  public Pose2d getRobotPose2d() {
    return new Pose2d(currentQuestPose.transformBy(ROBOT_TO_QUEST3D.inverse()).toPose2d().getTranslation(),gyroAngle.get().rotateBy(Rotation2d.fromDegrees(90)));// the transformBy is to switch x & y
  }

  // Check if Quest is connected
  public boolean isConnected() {
    return questNav.isConnected();
  }

  // Check if Quest is tracking
  public boolean isTracking() {
    return questNav.isTracking();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
      currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
      timestamp = poseFrames[poseFrames.length - 1].dataTimestamp();
      // Display Quest pose

      // the quest x & y are oppeset so i am switching (if it were more than +-90 than
      // i will had to transformBy)
      // never mind i will use transformby
      SmartDashboard.putNumber("Quest/X", getRobotPose2d().getX());
      SmartDashboard.putNumber("Quest/Y", getRobotPose2d().getY());
      // SmartDashboard.putNumber("Quest/angle", getRobotPose2d().getRotation().getDegrees());

      // field.setRobotPose(getRobotPose2d());
      field3.setRobotPose(currentQuestPose.plus(ROBOT_TO_QUEST3D.inverse()).toPose2d().rotateBy(gyroAngle.get().rotateBy(Rotation2d.fromDegrees(90))));
    }


    // Battery monitoring
    questNav.getBatteryPercent().ifPresent(
        battery -> SmartDashboard.putNumber("Quest Battery %", battery));
  }

  // gives me the timestamp of the newst frame
  public double getTimestamp() {
    return timestamp;
  }

  public void questReset() {
    questNav.setPose(Pose3d.kZero);
  }
  public void questResetfromRobotToQuest(){
    questNav.setPose(Pose3d.kZero.transformBy(ROBOT_TO_QUEST3D));
  }
}
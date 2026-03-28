package frc.demacia.vision.subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.geometry.Field2dDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Pose3dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Rotation3dDemacia;
import frc.demacia.utils.log.LogManager;
import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.demacia.vision.utils.VisionConstants.*;

public class Quest extends SubsystemBase {

  private Field2dDemacia robotField;

  private QuestNav questNav;
  private Pose3dDemacia currentQuestPose;
  private double timestamp;

  public Quest() {
    timestamp = 0;
    questNav = new QuestNav();
    questNav.commandPeriodic();

    robotField = new Field2dDemacia();// robot pose

    questNav.commandPeriodic();

    robotField = new Field2dDemacia();// robot pose

    currentQuestPose = new Pose3dDemacia(); // Initialize to origin - IMPORTANT!

    addLog();
  }

  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager.addEntry("Quest/Latency", questNav::getLatency).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);
    LogManager.addEntry("Quest/Battery", questNav::getBatteryPercent).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);
    // LogManager.addEntry("Quest/LibVersion",
    // questNav::getLibVersion).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP);

    SmartDashboard.putData("Quest/Reset Quest Pose",
        new InstantCommand(() -> RobotPose.getInstance().setQuestPose()).ignoringDisable(true));
    // SmartDashboard.putData("Quest/Field", field);
    SmartDashboard.putData("Quest/robotField", robotField);

  }

  // Set robot pose (transforms to Quest frame and sends to QuestNav)
  public void setQuestPose(Pose3dDemacia currentBotpose) {
    questNav.setPose(new Pose3d(currentBotpose.transformBy(ROBOT_TO_QUEST3D).toMatrix()));// the transformBy is to
                                                                                          // switch x & y and gives back
  } // the hight of the quest

  /**
   * * @return the center of the robot form quest
   */
  public Pose2dDemacia getRobotPose2d() {
    // return new
    // Pose2d(currentQuestPose.transformBy(ROBOT_TO_QUEST3D.inverse()).toPose2d().getTranslation(),gyroAngle.get().rotateBy(Rotation2d.fromDegrees(90)));//
    // the transformBy is to switch x & y
    return currentQuestPose.transformBy(ROBOT_TO_QUEST3D.inverse()).toPose2d();
  }

  // Check if Quest is connected
  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isWorking() {
    return isConnected() && isTracking() && questNav.getBatteryPercent().getAsInt() > 10;
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

      if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
        currentQuestPose = new Pose3dDemacia(poseFrames[poseFrames.length - 1].questPose3d().toMatrix());
        timestamp = poseFrames[poseFrames.length - 1].dataTimestamp();
        // Display Quest pose

        // the quest x & y

        // the quest x & y

        SmartDashboard.putNumber("Quest/X", getRobotPose2d().getX());
        SmartDashboard.putNumber("Quest/Y", getRobotPose2d().getY());

        robotField.setRobotPose(currentQuestPose.transformBy(ROBOT_TO_QUEST3D.inverse()).toPose2d());
      }
    }

    robotField.setRobotPose(currentQuestPose.transformBy(ROBOT_TO_QUEST3D.inverse()).toPose2d());
  }

  // gives me the timestamp of the newst frame
  public double getTimestamp() {
    return timestamp;
  }

  public void setHeading(Rotation2dDemacia angle) {
    setQuestPose(new Pose3dDemacia(getRobotPose2d().getX(), getRobotPose2d().getY(), currentQuestPose.getZ(),
        new Rotation3dDemacia(angle)));
  }
}
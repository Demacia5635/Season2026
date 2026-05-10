package frc.demacia.path.Trgectory;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChoreoBasedTrajectory extends Trajectory<SwerveSample> {
    // The trajectory is represented as a list of SwerveSample points. The trajectory will be followed by interpolating between these points.
    List<SwerveSample> points;
    List<EventMarker> events;
    int size = 0;

    EventLoop loop = new EventLoop();

    // data for the current segment of the trajectory. The current segment is defined by the two points that the robot is currently between. The current segment is updated as the robot moves along the trajectory.
    int currentIndex = -1;
    Translation2d from = null;
    Translation2d to = null;
    double dist = 0;
    Rotation2d heading = null;
    SwerveSample lastSample = null;
    Rotation2d currentToFrom = null;
    Rotation2d currentToTo = null;
    double lastEventTimestamp = 0;
    int nextEventIndex = 0;

    // supplier/consumer for the trajectory. The supplier is used to get the current pose of the robot, which is used to determine where the robot is on the trajectory and how to update the current segment of the trajectory. The consumer is used to output the current sample for the robot to follow, which is determined based on the current segment of the trajectory that the robot is on.
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<SwerveSample> outputConsumer;

    public ChoreoBasedTrajectory(String name, Supplier<Pose2d> poseSupplier, Consumer<SwerveSample> outputConsumer) {
        super(name, List.of(),List.of(), List.of());
        points = this.samples();
        events = this.events();
        loop = new EventLoop();
        if(events.size() > 0) {
            events.sort((a, b) -> Double.compare(((EventMarker)a).timestamp, ((EventMarker)b).timestamp)); // sort the events by their timestamp so that we can trigger them in the correct order as the robot follows the trajectory.
        }
        size = points.size();
        this.poseSupplier = poseSupplier;
        this.outputConsumer = outputConsumer;
    }

    /**
     * Reset the trajectory to the beginning. This will set the current segment to the first segment of the trajectory and reset all the data for the current segment. This should be called before starting to follow the trajectory.
     */
    public void reset() {
        currentIndex = 0;
        lastEventTimestamp = -1;
        nextEventIndex = 0;
        if(size == 0) { // No points in the trajectory, so we can't follow it. We will just set everything to null and return.
            from = null;
            to = null;
            heading = null;
            lastSample = null;
            dist = 0;
        } else if(size == 1) { // Only one point in the trajectory, so we will just set the current segment to that point and return. We will set the heading to null since we don't have a second point to calculate the heading from.
            from = points.get(0).getPose().getTranslation();
            to = from;
            heading = null;
            lastSample = points.get(0);
            dist = 0;
        } else { // We have at least two points in the trajectory, so we will set the current segment to the first two points and calculate the heading and distance for that segment.
            from = points.get(0).getPose().getTranslation();
            to = points.get(1).getPose().getTranslation();
            heading = to.minus(from).getAngle();
            lastSample = points.get(1);
            dist = to.minus(from).getNorm();
        }
    }

    // Check if the robot has reached the next point in the trajectory. If it has, update the current segment to the next segment and update all the data for the new current segment. This should be called every time we get a new sample from the robot's sensors to check if we need to update the current segment.
    private void checkNextSample(Translation2d currentPosition) {
        if(heading == null) { // We are at the end of the trajectory, so we don't need to check for the next sample. We will just set everything to null and
            return;
        }
        if(Math.abs(currentToTo.getDegrees()) > 90) { // We have reached the next point in the trajectory, so we will update the current segment to the next segment and update all the data for the new current segment.
            currentIndex++;
            if(currentIndex + 1 == size) { // We have reached the end of the trajectory, so we will set everything to null and return. We will set the heading to null since we don't have a second point to calculate the heading from.
                from = null;
                to = null;
                heading = null;
                dist = 0;
                return;
            } else { // We have not reached the end of the trajectory, so we will update the current segment to the next segment and update all the data for the new current segment.
                doCommandsTo(points.get(currentIndex+1).getTimestamp());
                from = to;
                to = points.get(currentIndex + 1).getPose().getTranslation();
                heading = to.minus(from).getAngle();
                lastSample = points.get(currentIndex + 1);
                currentToFrom = from.minus(currentPosition).getAngle().minus(heading);
                currentToTo = to.minus(currentPosition).getAngle().minus(heading);
                dist = to.minus(from).getNorm();
                checkNextSample(currentPosition); // We will check again to see if we have reached the next point in the trajectory, since it's possible that we have reached multiple points in the trajectory since the last time we checked. This will ensure that we are always on the correct segment of the trajectory.
            }
        }
    }

    // Check if the robot is before the current segment of the trajectory. This is used to determine if we should return the current sample or if we should return the next sample. If the robot is before the current segment, we will return the current sample. If the robot is after the current segment, we will return the next sample. This should be called every time we get a new sample from the robot's sensors to check if we should return the current sample or if we should return the next sample.
    private boolean isBeforeFrom(Translation2d currentPosition) {
        if(from == null || to == null || heading == null) { // We are at the end of the trajectory, so we will just return false since we don't have a current segment to compare to.
            return false;
        }
        if(Math.abs(currentToFrom.getDegrees()) < 90) {
            return true;
        }
        return false;
    }

    // Get the current sample for the robot to follow. This will return the current sample based on the current segment of the trajectory that the robot is on. If the robot is before the current segment, it will return the current sample. If the robot is after the current segment, it will return the next sample. This should be called every time we get a new sample from the robot's sensors to get the current sample for the robot to follow.
    public SwerveSample getSample(Pose2d currentPose) {
        if(to == null ) { // We are at the end of the trajectory, so we will just return the last sample since we don't have a current segment to compare to.
            if(lastSample != null) { // We have no samples in the trajectory, so we will just return null since we don't have anything to return.
                doCommandsTo(lastSample.getTimestamp());
            }
            return lastSample;
        }
        currentToFrom = from != null ? from.minus(currentPose.getTranslation()).getAngle().minus(heading) : currentPose.getRotation();
        currentToTo = to.minus(currentPose.getTranslation()).getAngle().minus(heading);
        checkNextSample(currentPose.getTranslation());
        if (isBeforeFrom(currentPose.getTranslation())) {
            return points.get(currentIndex);
        } else if(heading == null) { // We are at the end of the trajectory, so we will just return the last sample since we don't have a current segment to compare to.
            from = null;
            to = null;
            if(lastSample != null) { // We have no samples in the trajectory, so we will just return null since we don't have anything to return.
                doCommandsTo(lastSample.getTimestamp());
            }
            return lastSample;
        }
        // We are after the current segment of the trajectory, so we will return the next sample for the robot to follow. We will interpolate between the current sample and the next sample based on how far along the current segment of the trajectory the robot is. This will ensure that the robot follows the trajectory smoothly and doesn't just jump to the next sample when it reaches the next point in the trajectory.
        var s1 = points.get(currentIndex); // the start point of the current segment of the trajectory
        var s2 = points.get(currentIndex + 1); // the end point of the current segment of the trajectory
        double d = currentPose.getTranslation().getDistance(from)*currentToTo.getCos(); // the distance along the current segment of the trajectory that the robot is.
        double timeStamp = s1.t + (d/dist)*(s2.t - s1.t); // the timestamp for the current sample for the robot to follow, based on how far along the current segment of the trajectory the robot is.
        doCommandsTo(timeStamp);
        return s1.interpolate(s2, timeStamp);        
    }

    public void initialize() {
        reset();
    }
    public void execute() {
        var currentPose = poseSupplier.get();
        var sample = getSample(currentPose);
        outputConsumer.accept(sample);
    }

    public void end(boolean interrupted) {
        outputConsumer.accept(lastSample);
    }

    public boolean isFinished() {
        return to == null; // We are at the end of the trajectory, so we will just return true since we don't have a current segment to compare to.
    }

    public Command followCommand(Subsystem... requirements) {
        return new FunctionalCommand(this::initialize, this::execute, this::end, this::isFinished, requirements);   
    }   

    public void doCommandsTo(double timestamp) {
        if(timestamp > lastEventTimestamp) {
            lastEventTimestamp = timestamp;
            while(nextEventIndex < events.size() && events.get(nextEventIndex).timestamp <= timestamp) {
                doCommand(events.get(nextEventIndex).event);
                nextEventIndex++;
            }
        }
    }
    public void doCommand(String name) {
    }
}

package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Objects;

import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto;
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class ChassisSpeedsDemacia implements ProtobufSerializable {

    /** Velocity along the x-axis. (Fwd is +) */
    public double vxMetersPerSecond;

    /** Velocity along the y-axis. (Left is +) */
    public double vyMetersPerSecond;

    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double omegaRadiansPerSecond;

    /** ChassisSpeeds protobuf for serialization. */
    public static final ChassisSpeedsProto proto = new ChassisSpeedsProto();

    /** ChassisSpeeds struct for serialization. */
    public static final ChassisSpeedsStruct struct = new ChassisSpeedsStruct();

  /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
  public ChassisSpeedsDemacia() {}

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   */
  public ChassisSpeedsDemacia(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
    this.vyMetersPerSecond = vyMetersPerSecond;
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
  }

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   */
  public ChassisSpeedsDemacia(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    this(vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond));
  }

    /**
     * Creates a Twist2d from ChassisSpeeds.
     *
     * @param dtSeconds The duration of the timestep.
     * @return Twist2d.
     */
    public Twist2dDemacia toTwist2d(double dtSeconds) {
        return new Twist2dDemacia(
                vxMetersPerSecond * dtSeconds,
                vyMetersPerSecond * dtSeconds,
                omegaRadiansPerSecond * dtSeconds);
    }

    /**
     * Discretizes a continuous-time chassis speed.
     *
     * <p>
     * This function converts a continuous-time chassis speed into a discrete-time
     * one such that
     * when the discrete-time chassis speed is applied for one timestep, the robot
     * moves as if the
     * velocity components are independent (i.e., the robot moves v_x * dt along the
     * x-axis, v_y * dt
     * along the y-axis, and omega * dt around the z-axis).
     *
     * <p>
     * This is useful for compensating for translational skew when translating and
     * rotating a
     * holonomic (swerve or mecanum) drivetrain. However, scaling down the
     * ChassisSpeeds after
     * discretizing (e.g., when desaturating swerve module speeds) rotates the
     * direction of net motion
     * in the opposite direction of rotational velocity, introducing a different
     * translational skew
     * which is not accounted for by discretization.
     *
     * @param vxMetersPerSecond     Forward velocity.
     * @param vyMetersPerSecond     Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     * @param dtSeconds             The duration of the timestep the speeds should
     *                              be applied for.
     * @return Discretized ChassisSpeeds.
     */
    public static ChassisSpeedsDemacia discretize(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double dtSeconds) {
        // Construct the desired pose after a timestep, relative to the current pose.
        // The desired pose
        // has decoupled translation and rotation.
        var desiredDeltaPose = new Pose2dDemacia(
                vxMetersPerSecond * dtSeconds,
                vyMetersPerSecond * dtSeconds,
                new Rotation2dDemacia(omegaRadiansPerSecond * dtSeconds));

        // Find the chassis translation/rotation deltas in the robot frame that move the
        // robot from its
        // current pose to the desired pose
        var twist = Pose2dDemacia.kZero.log(desiredDeltaPose);

        // Turn the chassis translation/rotation deltas into average velocities
        return new ChassisSpeedsDemacia(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }

    /**
     * Discretizes a continuous-time chassis speed.
     *
     * <p>
     * This function converts a continuous-time chassis speed into a discrete-time
     * one such that
     * when the discrete-time chassis speed is applied for one timestep, the robot
     * moves as if the
     * velocity components are independent (i.e., the robot moves v_x * dt along the
     * x-axis, v_y * dt
     * along the y-axis, and omega * dt around the z-axis).
     *
     * <p>
     * This is useful for compensating for translational skew when translating and
     * rotating a
     * holonomic (swerve or mecanum) drivetrain. However, scaling down the
     * ChassisSpeeds after
     * discretizing (e.g., when desaturating swerve module speeds) rotates the
     * direction of net motion
     * in the opposite direction of rotational velocity, introducing a different
     * translational skew
     * which is not accounted for by discretization.
     *
     * @param vx    Forward velocity.
     * @param vy    Sideways velocity.
     * @param omega Angular velocity.
     * @param dt    The duration of the timestep the speeds should be applied for.
     * @return Discretized ChassisSpeeds.
     */
    public static ChassisSpeedsDemacia discretize(
            LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Time dt) {
        return discretize(
                vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), dt.in(Seconds));
    }

    /**
     * Discretizes a continuous-time chassis speed.
     *
     * <p>
     * This function converts a continuous-time chassis speed into a discrete-time
     * one such that
     * when the discrete-time chassis speed is applied for one timestep, the robot
     * moves as if the
     * velocity components are independent (i.e., the robot moves v_x * dt along the
     * x-axis, v_y * dt
     * along the y-axis, and omega * dt around the z-axis).
     *
     * <p>
     * This is useful for compensating for translational skew when translating and
     * rotating a
     * holonomic (swerve or mecanum) drivetrain. However, scaling down the
     * ChassisSpeeds after
     * discretizing (e.g., when desaturating swerve module speeds) rotates the
     * direction of net motion
     * in the opposite direction of rotational velocity, introducing a different
     * translational skew
     * which is not accounted for by discretization.
     *
     * @param continuousSpeeds The continuous speeds.
     * @param dtSeconds        The duration of the timestep the speeds should be
     *                         applied for.
     * @return Discretized ChassisSpeeds.
     */
    public static ChassisSpeedsDemacia discretize(ChassisSpeedsDemacia continuousSpeeds, double dtSeconds) {
        return discretize(
                continuousSpeeds.vxMetersPerSecond,
                continuousSpeeds.vyMetersPerSecond,
                continuousSpeeds.omegaRadiansPerSecond,
                dtSeconds);
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative
     * ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond     The component of speed in the x direction
     *                              relative to the field.
     *                              Positive x is away from your alliance wall.
     * @param vyMetersPerSecond     The component of speed in the y direction
     *                              relative to the field.
     *                              Positive y is to your left when standing behind
     *                              the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle            The angle of the robot as measured by a
     *                              gyroscope. The robot's angle is
     *                              considered to be zero when it is facing directly
     *                              away from your alliance station wall.
     *                              Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2dDemacia robotAngle) {
        // CW rotation into chassis frame
        var rotated = new Translation2dDemacia(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
        return new ChassisSpeedsDemacia(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative
     * ChassisSpeeds
     * object.
     *
     * @param vx         The component of speed in the x direction relative to the
     *                   field. Positive x is away
     *                   from your alliance wall.
     * @param vy         The component of speed in the y direction relative to the
     *                   field. Positive y is to
     *                   your left when standing behind the alliance wall.
     * @param omega      The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The
     *                   robot's angle is
     *                   considered to be zero when it is facing directly away from
     *                   your alliance station wall.
     *                   Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromFieldRelativeSpeeds(
            LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2dDemacia robotAngle) {
        return fromFieldRelativeSpeeds(
                vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
    }

    /**
     * Converts a user provided field-relative ChassisSpeeds object into a
     * robot-relative
     * ChassisSpeeds object.
     *
     * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds
     *                            in the field frame
     *                            of reference. Positive x is away from your
     *                            alliance wall. Positive y is to your left when
     *                            standing behind the alliance wall.
     * @param robotAngle          The angle of the robot as measured by a gyroscope.
     *                            The robot's angle is
     *                            considered to be zero when it is facing directly
     *                            away from your alliance station wall.
     *                            Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromFieldRelativeSpeeds(
            ChassisSpeedsDemacia fieldRelativeSpeeds, Rotation2dDemacia robotAngle) {
        return fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond,
                fieldRelativeSpeeds.omegaRadiansPerSecond,
                robotAngle);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative
     * ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond     The component of speed in the x direction
     *                              relative to the robot.
     *                              Positive x is towards the robot's front.
     * @param vyMetersPerSecond     The component of speed in the y direction
     *                              relative to the robot.
     *                              Positive y is towards the robot's left.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle            The angle of the robot as measured by a
     *                              gyroscope. The robot's angle is
     *                              considered to be zero when it is facing directly
     *                              away from your alliance station wall.
     *                              Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromRobotRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2dDemacia robotAngle) {
        // CCW rotation out of chassis frame
        var rotated = new Translation2dDemacia(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle);
        return new ChassisSpeedsDemacia(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided robot-relative set of speeds into a field-relative
     * ChassisSpeeds
     * object.
     *
     * @param vx         The component of speed in the x direction relative to the
     *                   robot. Positive x is
     *                   towards the robot's front.
     * @param vy         The component of speed in the y direction relative to the
     *                   robot. Positive y is
     *                   towards the robot's left.
     * @param omega      The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The
     *                   robot's angle is
     *                   considered to be zero when it is facing directly away from
     *                   your alliance station wall.
     *                   Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromRobotRelativeSpeeds(
            LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2dDemacia robotAngle) {
        return fromRobotRelativeSpeeds(
                vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
    }

    /**
     * Converts a user provided robot-relative ChassisSpeeds object into a
     * field-relative
     * ChassisSpeeds object.
     *
     * @param robotRelativeSpeeds The ChassisSpeeds object representing the speeds
     *                            in the robot frame
     *                            of reference. Positive x is towards the robot's
     *                            front. Positive y is towards the robot's
     *                            left.
     * @param robotAngle          The angle of the robot as measured by a gyroscope.
     *                            The robot's angle is
     *                            considered to be zero when it is facing directly
     *                            away from your alliance station wall.
     *                            Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of
     *         reference.
     */
    public static ChassisSpeedsDemacia fromRobotRelativeSpeeds(
            ChassisSpeedsDemacia robotRelativeSpeeds, Rotation2dDemacia robotAngle) {
        return fromRobotRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                robotAngle);
    }

    /**
     * Adds two ChassisSpeeds and returns the sum.
     *
     * <p>
     * For example, ChassisSpeeds{1.0, 0.5, 0.75} + ChassisSpeeds{2.0, 1.5, 0.25} =
     * ChassisSpeeds{3.0, 2.0, 1.0}
     *
     * @param other The ChassisSpeeds to add.
     * @return The sum of the ChassisSpeeds.
     */
    public ChassisSpeedsDemacia plus(ChassisSpeedsDemacia other) {
        return new ChassisSpeedsDemacia(
                vxMetersPerSecond + other.vxMetersPerSecond,
                vyMetersPerSecond + other.vyMetersPerSecond,
                omegaRadiansPerSecond + other.omegaRadiansPerSecond);
    }

    /**
     * Subtracts the other ChassisSpeeds from the current ChassisSpeeds and returns
     * the difference.
     *
     * <p>
     * For example, ChassisSpeeds{5.0, 4.0, 2.0} - ChassisSpeeds{1.0, 2.0, 1.0} =
     * ChassisSpeeds{4.0, 2.0, 1.0}
     *
     * @param other The ChassisSpeeds to subtract.
     * @return The difference between the two ChassisSpeeds.
     */
    public ChassisSpeedsDemacia minus(ChassisSpeedsDemacia other) {
        return new ChassisSpeedsDemacia(
                vxMetersPerSecond - other.vxMetersPerSecond,
                vyMetersPerSecond - other.vyMetersPerSecond,
                omegaRadiansPerSecond - other.omegaRadiansPerSecond);
    }

    /**
     * Returns the inverse of the current ChassisSpeeds. This is equivalent to
     * negating all components
     * of the ChassisSpeeds.
     *
     * @return The inverse of the current ChassisSpeeds.
     */
    public ChassisSpeedsDemacia unaryMinus() {
        return new ChassisSpeedsDemacia(-vxMetersPerSecond, -vyMetersPerSecond, -omegaRadiansPerSecond);
    }

    /**
     * Multiplies the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>
     * For example, ChassisSpeeds{2.0, 2.5, 1.0} * 2 = ChassisSpeeds{4.0, 5.0, 1.0}
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled ChassisSpeeds.
     */
    public ChassisSpeedsDemacia times(double scalar) {
        return new ChassisSpeedsDemacia(
                vxMetersPerSecond * scalar, vyMetersPerSecond * scalar, omegaRadiansPerSecond * scalar);
    }

    /**
     * Divides the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>
     * For example, ChassisSpeeds{2.0, 2.5, 1.0} / 2 = ChassisSpeeds{1.0, 1.25, 0.5}
     *
     * @param scalar The scalar to divide by.
     * @return The scaled ChassisSpeeds.
     */
    public ChassisSpeedsDemacia div(double scalar) {
        return new ChassisSpeedsDemacia(
                vxMetersPerSecond / scalar, vyMetersPerSecond / scalar, omegaRadiansPerSecond / scalar);
    }

    @Override
    public final int hashCode() {
        return Objects.hash(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    @Override
    public boolean equals(Object o) {
        return o == this
                || o instanceof ChassisSpeedsDemacia c
                        && vxMetersPerSecond == c.vxMetersPerSecond
                        && vyMetersPerSecond == c.vyMetersPerSecond
                        && omegaRadiansPerSecond == c.omegaRadiansPerSecond;
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
                vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}

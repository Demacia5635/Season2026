package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Objects;

import edu.wpi.first.math.kinematics.proto.SwerveModuleStateProto;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.protobuf.ProtobufSerializable;

public class SwerveModuleStateDemacia implements Comparable<SwerveModuleStateDemacia>, ProtobufSerializable {
    /** Speed of the wheel of the module. */
    public double speedMetersPerSecond;

    /** Angle of the module. */
    public Rotation2dDemacia angle = Rotation2dDemacia.kZero;

    /** SwerveModuleState protobuf for serialization. */
    public static final SwerveModuleStateProto proto = new SwerveModuleStateProto();

    /** SwerveModuleState struct for serialization. */
    public static final SwerveModuleStateStruct struct = new SwerveModuleStateStruct();

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleStateDemacia() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleStateDemacia(double speedMetersPerSecond, Rotation2dDemacia angle) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speed The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleStateDemacia(LinearVelocity speed, Rotation2dDemacia angle) {
    this(speed.in(MetersPerSecond), angle);
  }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof SwerveModuleStateDemacia other
                && Math.abs(other.speedMetersPerSecond - speedMetersPerSecond) < 1E-9
                && angle.equals(other.angle);
    }

    @Override
    public int hashCode() {
        return Objects.hash(speedMetersPerSecond, angle);
    }

    /**
     * Compares two swerve module states. One swerve module is "greater" than the
     * other if its speed
     * is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModuleStateDemacia other) {
        return Double.compare(this.speedMetersPerSecond, other.speedMetersPerSecond);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speedMetersPerSecond, angle);
    }

    /**
     * Minimize the change in heading this swerve module state would require by
     * potentially reversing
     * the direction the wheel spins. If this is used with the PIDController class's
     * continuous input
     * functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param currentAngle The current module angle.
     */
    public void optimize(Rotation2dDemacia currentAngle) {
        var delta = angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            speedMetersPerSecond *= -1;
            angle = angle.rotateBy(Rotation2dDemacia.kPi);
        }
    }

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. If this is used with the
     * PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90
     * degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     * @deprecated Use the instance method instead.
     */
    @Deprecated
    public static SwerveModuleStateDemacia optimize(
            SwerveModuleStateDemacia desiredState, Rotation2dDemacia currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleStateDemacia(
                    -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2dDemacia.kPi));
        } else {
            return new SwerveModuleStateDemacia(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    /**
     * Scales speed by cosine of angle error. This scales down movement
     * perpendicular to the desired
     * direction of travel that can occur when modules change directions. This
     * results in smoother
     * driving.
     *
     * @param currentAngle The current module angle.
     */
    public void cosineScale(Rotation2dDemacia currentAngle) {
        speedMetersPerSecond *= angle.minus(currentAngle).getCos();
    }
}

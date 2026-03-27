package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.proto.SwerveModulePositionProto;
import edu.wpi.first.math.kinematics.struct.SwerveModulePositionStruct;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class SwerveModulePositionDemacia implements Comparable<SwerveModulePositionDemacia>,
        Interpolatable<SwerveModulePositionDemacia>, ProtobufSerializable, StructSerializable {
    /** Distance measured by the wheel of the module. */
    public double distanceMeters;

    /** Angle of the module. */
    public Rotation2dDemacia angle = Rotation2dDemacia.kZero;

    /** SwerveModulePosition protobuf for serialization. */
    public static final SwerveModulePositionProto proto = new SwerveModulePositionProto();

    /** SwerveModulePosition struct for serialization. */
    public static final SwerveModulePositionStruct struct = new SwerveModulePositionStruct();

  /** Constructs a SwerveModulePosition with zeros for distance and angle. */
  public SwerveModulePositionDemacia() {}

  /**
   * Constructs a SwerveModulePosition.
   *
   * @param distanceMeters The distance measured by the wheel of the module.
   * @param angle The angle of the module.
   */
public SwerveModulePositionDemacia(double distanceMeters, Rotation2dDemacia angle) {
    this.distanceMeters = distanceMeters;
    this.angle = angle;
  }

  /**
   * Constructs a SwerveModulePosition.
   *
   * @param distance The distance measured by the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModulePositionDemacia(Distance distance, Rotation2dDemacia angle) {
    this(distance.in(Meters), angle);
  }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof SwerveModulePositionDemacia other
                && Math.abs(other.distanceMeters - distanceMeters) < 1E-9
                && angle.equals(other.angle);
    }

    @Override
    public int hashCode() {
        return Objects.hash(distanceMeters, angle);
    }

    /**
     * Compares two swerve module positions. One swerve module is "greater" than the
     * other if its
     * distance is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModulePositionDemacia other) {
        return Double.compare(this.distanceMeters, other.distanceMeters);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModulePosition(Distance: %.2f m, Angle: %s)", distanceMeters, angle);
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePositionDemacia copy() {
        return new SwerveModulePositionDemacia(distanceMeters, angle);
    }

    @Override
    public SwerveModulePositionDemacia interpolate(SwerveModulePositionDemacia endValue, double t) {
        return new SwerveModulePositionDemacia(
                MathUtil.interpolate(this.distanceMeters, endValue.distanceMeters, t),
                this.angle.interpolate(endValue.angle, t));
    }
}

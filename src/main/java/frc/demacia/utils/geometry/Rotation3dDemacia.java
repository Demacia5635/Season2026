package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.Radians;

import java.util.Objects;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.proto.Rotation3dProto;
import edu.wpi.first.math.geometry.struct.Rotation3dStruct;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Rotation3dDemacia implements Interpolatable<Rotation3dDemacia>, ProtobufSerializable, StructSerializable {
    public static Rotation3dDemacia kZero = new Rotation3dDemacia();

    private QuaternionDemacia m_q;

    public Rotation3dDemacia() {
        m_q = new QuaternionDemacia();
    }

    public Rotation3dDemacia(QuaternionDemacia m_q) {
        this.m_q = m_q.normalize();
    }

    public Rotation3dDemacia(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);

        m_q = new QuaternionDemacia(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy);
    }

    public Rotation3dDemacia(Angle roll, Angle pitch, Angle yaw) {
        this(roll.in(Radians), pitch.in(Radians), yaw.in(Radians));
    }

    public Rotation3dDemacia(Vector<N3> rvec) {
        this(rvec, rvec.norm());
    }

    public Rotation3dDemacia(Vector<N3> axis, double angleRadians) {
        double norm = axis.norm();
        if (norm == 0) {
            m_q = new QuaternionDemacia();
            return;
        }

        var v = axis.times(1 / norm).times(Math.sin(angleRadians / 2));
        m_q = new QuaternionDemacia(Math.cos(angleRadians / 2), v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Rotation3dDemacia(Vector<N3> axis, Angle angle) {
        this(axis, angle.in(Radians));
    }

    public Rotation3dDemacia(Matrix<N3, N3> rotationMatrix) {
        final var R = rotationMatrix;

        if (R.times(R.transpose()).minus(Matrix.eye(Nat.N3())).normF() > 1e-9) {
            m_q = new QuaternionDemacia();
            return;
        }
        if (Math.abs(R.det() - 1) > 1e-9) {
            m_q = new QuaternionDemacia();
            return;
        }

        double trace = R.get(0, 0) + R.get(1, 1) + R.get(2, 2);
        double w;
        double x;
        double y;
        double z;

        if (trace > 0) {
            double s = 0.5 / Math.sqrt(trace + 1);
            w = 0.25 / s;
            x = (R.get(2, 1) - R.get(1, 2)) * s;
            y = (R.get(0, 2) - R.get(2, 0)) * s;
            z = (R.get(1, 0) - R.get(0, 1)) * s;
        } else {
            if (R.get(0, 0) > R.get(1, 1) && R.get(0, 0) > R.get(2, 2)) {
                double s = 2 * Math.sqrt(1 + R.get(0, 0) - R.get(2, 2));
                w = (R.get(2, 1) - R.get(1, 2)) / s;
                x = 0.25 * s;
                y = (R.get(0, 1) + R.get(1, 0)) / s;
                z = (R.get(0, 2) + R.get(2, 0)) / s;
            } else if (R.get(1, 1) > R.get(2, 2)) {
                double s = 2.0 * Math.sqrt(1.0 + R.get(1, 1) - R.get(0, 0) - R.get(2, 2));
                w = (R.get(0, 2) - R.get(2, 0)) / s;
                x = (R.get(0, 1) + R.get(1, 0)) / s;
                y = 0.25 * s;
                z = (R.get(1, 2) + R.get(2, 1)) / s;
            } else {
                double s = 2.0 * Math.sqrt(1.0 + R.get(2, 2) - R.get(0, 0) - R.get(1, 1));
                w = (R.get(1, 0) - R.get(0, 1)) / s;
                x = (R.get(0, 2) + R.get(2, 0)) / s;
                y = (R.get(1, 2) + R.get(2, 1)) / s;
                z = 0.25 * s;
            }
        }

        m_q = new QuaternionDemacia(w, x, y, z);
    }

    public Rotation3dDemacia(Vector<N3> initial, Vector<N3> last) {
        double dot = initial.dot(last);
        double normProduct = initial.norm() * last.norm();
        double dotNorm = dot / normProduct;

        if (dotNorm > 1 - 1E-9) {
            m_q = new QuaternionDemacia();
        } else if (dotNorm < -1 + 1E-9) {
            double x = Math.abs(initial.get(0, 0));
            double y = Math.abs(initial.get(1, 0));
            double z = Math.abs(initial.get(2, 0));

            // Find vector that is most orthogonal to initial vector
            Vector<N3> other;
            if (x < y) {
                if (x < z) {
                    // Use x-axis
                    other = VecBuilder.fill(1, 0, 0);
                } else {
                    // Use z-axis
                    other = VecBuilder.fill(0, 0, 1);
                }
            } else {
                if (y < z) {
                    // Use y-axis
                    other = VecBuilder.fill(0, 1, 0);
                } else {
                    // Use z-axis
                    other = VecBuilder.fill(0, 0, 1);
                }
            }

            var axis = Vector.cross(initial, other);

            double axisNorm = axis.norm();
            m_q = new QuaternionDemacia(
                    0.0, axis.get(0, 0) / axisNorm, axis.get(1, 0) / axisNorm, axis.get(2, 0) / axisNorm);
        } else {
            var axis = Vector.cross(initial, last);

            // https://stackoverflow.com/a/11741520
            m_q = new QuaternionDemacia(normProduct + dot, axis.get(0, 0), axis.get(1, 0), axis.get(2, 0))
                    .normalize();
        }
    }

    public Rotation3dDemacia(Rotation2dDemacia rotation) {
        this(0, 0, rotation.getRadians());
    }

    public Rotation3dDemacia plus(Rotation3dDemacia other) {
        return rotateBy(other);
    }

    public Rotation3dDemacia minus(Rotation3dDemacia other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation3dDemacia unaryMinus() {
        m_q.inverse();
        return this;
    }

    public Rotation3dDemacia times(double scalar) {
        if (m_q.getW() >= 0) {
            var vector = VecBuilder.fill(m_q.getX(), m_q.getY(), m_q.getZ());
            double angleRadians = 2 * scalar * Math.acos(m_q.getW());
            var v = vector.times(1 / vector.norm()).times(Math.sin(angleRadians / 2));
            m_q.setW(Math.cos(angleRadians / 2));
            m_q.setX(v.get(0, 0));
            m_q.setY(v.get(1, 0));
            m_q.setZ(v.get(2, 0));
        } else {
            var vector = VecBuilder.fill(-m_q.getX(), -m_q.getY(), -m_q.getZ());
            double angleRadians = 2 * scalar * Math.acos(-m_q.getW());
            var v = vector.times(1 / vector.norm()).times(Math.sin(angleRadians / 2));
            m_q.setW(Math.cos(angleRadians / 2));
            m_q.setX(v.get(0, 0));
            m_q.setY(v.get(1, 0));
            m_q.setZ(v.get(2, 0));
        }
        return this;
    }

    public Rotation3dDemacia div(double scalar) {
        return times(1 / scalar);
    }

    public Rotation3dDemacia rotateBy(Rotation3dDemacia other) {
        m_q.times(other.m_q);
        return this;
    }

    public Rotation3dDemacia relativeTo(Rotation3dDemacia other) {
        m_q = other.m_q.inverse().times(m_q);
        return this;
    }

    public QuaternionDemacia getQuaternion() {
        return m_q;
    }

    public void setQuaternion(double w, double x, double y, double z) {
        m_q.setW(w);
        m_q.setX(x);
        m_q.setY(y);
        m_q.setZ(z);
    }

    public void setQuaternion(QuaternionDemacia quaternion) {
        m_q = quaternion;
    }
    
    public double getX() {
        final var w = m_q.getW();
        final var x = m_q.getX();
        final var y = m_q.getY();
        final var z = m_q.getZ();

        final var cxcy = 1 - 2 * (x * x + y * y);
        final var sxcy = 2 * (w * x + y * z);
        final var cy_sq = cxcy * cxcy + sxcy * sxcy;
        if (cy_sq > 1e-20) {
            return Math.atan2(sxcy, cxcy);
        } else {
            return 0;
        }
    }

    public double getY() {
        
    final var w = m_q.getW();
    final var x = m_q.getX();
    final var y = m_q.getY();
    final var z = m_q.getZ();

    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
    double ratio = 2.0 * (w * y - z * x);
    if (Math.abs(ratio) >= 1.0) {
      return Math.copySign(Math.PI / 2.0, ratio);
    } else {
      return Math.asin(ratio);
    }
  }

  /**
   * Returns the counterclockwise rotation angle around the Z axis (yaw) in radians.
   *
   * @return The counterclockwise rotation angle around the Z axis (yaw) in radians.
   */
  public double getZ() {
    final var w = m_q.getW();
    final var x = m_q.getX();
    final var y = m_q.getY();
    final var z = m_q.getZ();

    // wpimath/algorithms.md
    final var cycz = 1.0 - 2.0 * (y * y + z * z);
    final var cysz = 2.0 * (w * z + x * y);
    final var cy_sq = cycz * cycz + cysz * cysz;
    if (cy_sq > 1e-20) {
      return Math.atan2(cysz, cycz);
    } else {
      return Math.atan2(2.0 * w * z, w * w - z * z);
    }
  }

  /**
   * Returns the counterclockwise rotation angle around the X axis (roll) in a measure.
   *
   * @return The counterclockwise rotation angle around the x axis (roll) in a measure.
   */
  public Angle getMeasureX() {
    return Radians.of(getX());
  }

  /**
   * Returns the counterclockwise rotation angle around the Y axis (pitch) in a measure.
   *
   * @return The counterclockwise rotation angle around the y axis (pitch) in a measure.
   */
  public Angle getMeasureY() {
    return Radians.of(getY());
  }

  /**
   * Returns the counterclockwise rotation angle around the Z axis (yaw) in a measure.
   *
   * @return The counterclockwise rotation angle around the z axis (yaw) in a measure.
   */
  public Angle getMeasureZ() {
    return Radians.of(getZ());
  }

  /**
   * Returns the axis in the axis-angle representation of this rotation.
   *
   * @return The axis in the axis-angle representation.
   */
  public Vector<N3> getAxis() {
    double norm =
        Math.sqrt(m_q.getX() * m_q.getX() + m_q.getY() * m_q.getY() + m_q.getZ() * m_q.getZ());
    if (norm == 0.0) {
      return VecBuilder.fill(0.0, 0.0, 0.0);
    } else {
      return VecBuilder.fill(m_q.getX() / norm, m_q.getY() / norm, m_q.getZ() / norm);
    }
  }

  /**
   * Returns the angle in radians in the axis-angle representation of this rotation.
   *
   * @return The angle in radians in the axis-angle representation of this rotation.
   */
  public double getAngle() {
    double norm =
        Math.sqrt(m_q.getX() * m_q.getX() + m_q.getY() * m_q.getY() + m_q.getZ() * m_q.getZ());
    return 2.0 * Math.atan2(norm, m_q.getW());
  }

  /**
   * Returns rotation matrix representation of this rotation.
   *
   * @return Rotation matrix representation of this rotation.
   */
  public Matrix<N3, N3> toMatrix() {
    double w = m_q.getW();
    double x = m_q.getX();
    double y = m_q.getY();
    double z = m_q.getZ();

    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N3(),
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y));
  }

  /**
   * Returns rotation vector representation of this rotation.
   *
   * @return Rotation vector representation of this rotation.
   */
  public Vector<N3> toVector() {
    return m_q.toRotationVector();
  }

  /**
   * Returns the angle in a measure in the axis-angle representation of this rotation.
   *
   * @return The angle in a measure in the axis-angle representation of this rotation.
   */
  public Angle getMeasureAngle() {
    return Radians.of(getAngle());
  }

  /**
   * Returns a Rotation2d representing this Rotation3d projected into the X-Y plane.
   *
   * @return A Rotation2d representing this Rotation3d projected into the X-Y plane.
   */
  public Rotation2dDemacia toRotation2d() {
    return new Rotation2dDemacia(getZ());
  }

  @Override
  public String toString() {
    return String.format("Rotation3d(%s)", m_q);
  }

  /**
   * Checks equality between this Rotation3d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    return obj instanceof Rotation3dDemacia other
        && Math.abs(Math.abs(m_q.dot(other.m_q)) - m_q.norm() * other.m_q.norm()) < 1e-9;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_q);
  }

  @Override
  public Rotation3dDemacia interpolate(Rotation3dDemacia endValue, double t) {
    return endValue.minus(this).times(MathUtil.clamp(t, 0, 1)).plus(this);
  }

  /** Rotation3d protobuf for serialization. */
  public static final Rotation3dProto proto = new Rotation3dProto();

  /** Rotation3d struct for serialization. */
  public static final Rotation3dStruct struct = new Rotation3dStruct();
}

package frc.demacia.utils.geometry;

import java.util.Objects;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.proto.QuaternionProto;
import edu.wpi.first.math.geometry.struct.QuaternionStruct;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class QuaternionDemacia implements ProtobufSerializable, StructSerializable {

    public static final QuaternionDemacia kZero = new QuaternionDemacia();

    private double m_w;
    private double m_x;
    private double m_y;
    private double m_z;

    public QuaternionDemacia() {
        m_w = 1;
        m_x = 0;
        m_y = 0;
        m_z = 0;
    }

    public QuaternionDemacia(double m_w, double m_x, double m_y, double m_z) {
        this.m_w = m_w;
        this.m_x = m_x;
        this.m_y = m_y;
        this.m_z = m_z;
    }

    public QuaternionDemacia plus(QuaternionDemacia other) {
        m_w += other.m_w;
        m_x += other.m_x;
        m_y += other.m_y;
        m_z += other.m_z;
        return this;
    }

    public QuaternionDemacia minus(QuaternionDemacia other) {
        m_w -= other.m_w;
        m_x -= other.m_x;
        m_y -= other.m_y;
        m_z -= other.m_z;
        return this;
    }

    public QuaternionDemacia divide(double scalar) {
        m_w /= scalar;
        m_x /= scalar;
        m_y /= scalar;
        m_z /= scalar;
        return this;
    }

    public QuaternionDemacia times(double scalar) {
        m_w *= scalar;
        m_x *= scalar;
        m_y *= scalar;
        m_z *= scalar;
        return this;
    }

    public QuaternionDemacia times(QuaternionDemacia other) {
        final var r1 = m_w;
        final var r2 = other.m_w;

        double dot = m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;

        double cross_x = m_y * other.m_z - other.m_y * m_z;
        double cross_y = other.m_x * m_z - m_x * other.m_z;
        double cross_z = m_x * other.m_y - other.m_x * m_y;

        m_w = r1 * r2 - dot;
        m_x = r1 * other.m_x + r2 * m_x + cross_x;
        m_y = r1 * other.m_y + r2 * m_y + cross_y;
        m_z = r1 * other.m_z + r2 * m_z + cross_z;
        return this;
    }

    @Override
    public String toString() {
        return String.format("QuanternionDemacia(%s, %s, %s, %s)", m_w, m_x, m_y, m_z);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof QuaternionDemacia otherDemacia
            && Math.abs(dot(otherDemacia) - norm() * otherDemacia.norm()) < 1e-9
            && Math.abs(norm() - otherDemacia.norm()) < 1e-9;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_w, m_x, m_y, m_z);
    }

    public QuaternionDemacia conjugate() {
        m_x = -m_x;
        m_y = -m_y;
        m_z = -m_z;
        return this;
    }

    public double dot(final QuaternionDemacia other) {
        return m_w * other.m_w
            + m_x * other.m_x
            + m_y * other.m_y
            + m_z * other.m_z;
    }

    public QuaternionDemacia inverse() {
        double norm = norm();
        return conjugate().divide(norm * norm);
    }

    public double norm() {
        return Math.sqrt(dot(this));
    }

    public QuaternionDemacia normalize() {
        double norm = norm();
        if (norm == 0) {
            return QuaternionDemacia.kZero;
        } else {
            m_w /= norm;
            m_x /= norm;
            m_y /= norm;
            m_z /= norm;
            return this;
        }
    }

    public QuaternionDemacia pow(double t) {
        return log().times(t).exp();
    }

    public QuaternionDemacia exp(QuaternionDemacia adjustment) {
        return adjustment.exp().times(this);
    }

    public QuaternionDemacia exp() {
        var scalar = Math.exp(m_w);

        var axial_magnitude = Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
        var cosine = Math.cos(axial_magnitude);

        double axial_scalar;

        if (axial_magnitude < 1e-9) {
            var axial_magnitude_sq = axial_magnitude * axial_magnitude;
            var axial_magnitude_sq_sq = axial_magnitude_sq * axial_magnitude_sq;
            axial_scalar = 1 - axial_magnitude_sq / 6 + axial_magnitude_sq_sq / 120;
        } else {
            axial_scalar = Math.sin(axial_magnitude) / axial_magnitude;
        }

        m_w = cosine * scalar;
        m_x *= axial_scalar * scalar;
        m_y *= axial_scalar * scalar;
        m_z *= axial_scalar * scalar;
        return this;
    }

    public QuaternionDemacia log(QuaternionDemacia end) {
        return end.times(inverse()).log();
    }

    public QuaternionDemacia log() {
        var norm = norm();
        var scalar = Math.log(norm);

        var v_norm = Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);

        var s_norm = m_w / norm;

        if (Math.abs(s_norm + 1) < 1e-9) {
            m_w = scalar;
            m_x = -Math.PI;
            m_y = 0;
            m_z = 0;
            return this;
        }

        double v_scalar;

        if (v_norm < 1e-9) {
            v_scalar = 1 / m_w - 1 / 3 * v_norm * v_norm / (m_w * m_w * m_w);
        } else {
            v_scalar = Math.atan2(v_norm, m_w) / v_norm;
        }

        m_w = scalar;
        m_x *= v_scalar;
        m_y *= v_scalar;
        m_z *= v_scalar;
        return this;
    }

    public double getW() {
        return m_w;
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }

    public double getZ() {
        return m_z;
    }

    public static QuaternionDemacia fromRotationVector(Vector<N3> rvec) {
        double theta = rvec.norm();

        double cos = Math.cos(theta / 2);

        double axial_scalar;

        if (theta < 1e-9) {
            axial_scalar = 1d / 2d - theta * theta / 48;
        } else {
            axial_scalar = Math.signum(theta / 2) / theta;
        }

        return new QuaternionDemacia(
            cos,
            axial_scalar * rvec.get(0, 0),
            axial_scalar * rvec.get(1, 0),
            axial_scalar * rvec.get(2, 0)
        );
    }

    public Vector<N3> toRotationVector() {
        double norm = Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);

        double coeff;
        if (norm < 1e-9) {
            coeff = 2 / m_w - 2d / 3d * norm * norm / (m_w * m_w * m_w);
        } else {
            if (m_w < 0) {
                coeff = 2 * Math.atan2(-norm, -m_w) / norm;
            } else {
                coeff = 2 * Math.atan2(norm, m_w) / norm;
            }
        }

        return VecBuilder.fill(coeff * m_x, coeff * m_y, coeff * m_z);
    }

    public static final QuaternionProto proto = new QuaternionProto();
    public static final QuaternionStruct struct = new QuaternionStruct();

    public void setW(double m_w) {
        this.m_w = m_w;
    }

    public void setX(double m_x) {
        this.m_x = m_x;
    }

    public void setY(double m_y) {
        this.m_y = m_y;
    }

    public void setZ(double m_z) {
        this.m_z = m_z;
    }
}

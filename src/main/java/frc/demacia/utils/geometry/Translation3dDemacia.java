package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.Meters;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.proto.Translation3dProto;
import edu.wpi.first.math.geometry.struct.Translation3dStruct;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Translation3dDemacia
        implements Interpolatable<Translation3dDemacia>, ProtobufSerializable, StructSerializable {
    public static final Translation3dDemacia kZero = new Translation3dDemacia();

    private double m_x;
    private double m_y;
    private double m_z;

    public Translation3dDemacia() {
        this(0, 0, 0);
    }

    public Translation3dDemacia(double m_x, double m_y, double m_z) {
        this.m_x = m_x;
        this.m_y = m_y;
        this.m_z = m_z;
    }

    public Translation3dDemacia(double distance, Rotation3dDemacia angle) {
        m_x = distance;
        m_y = 0;
        m_z = 0;
        rotateBy(angle);
    }

    public Translation3dDemacia(Distance x, Distance y, Distance z) {
        this(x.in(Meters), y.in(Meters), z.in(Meters));
    }

    public Translation3dDemacia(Translation2dDemacia translation) {
        this(translation.getX(), translation.getY(), 0);
    }

    public Translation3dDemacia(Vector<N3> vector) {
        this(vector.get(0), vector.get(1), vector.get(2));
    }

    public double getDistance(Translation3dDemacia other) {
        double dx = other.m_x - m_x;
        double dy = other.m_y - m_y;
        double dz = other.m_z - m_z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public double getSquaredDistance(Translation3dDemacia other) {
        double dx = other.m_x - m_x;
        double dy = other.m_y - m_y;
        double dz = other.m_z - m_z;
        return dx * dx + dy * dy + dz * dz;
    }

    public double getX() {
        return m_x;
    }

    public void setX(double m_x) {
        this.m_x = m_x;
    }

    public double getY() {
        return m_y;
    }

    public void setY(double m_y) {
        this.m_y = m_y;
    }

    public double getZ() {
        return m_z;
    }

    public void setZ(double m_z) {
        this.m_z = m_z;
    }

    /**
     * Returns the X component of the translation in a measure.
     *
     * @return The x component of the translation in a measure.
     */
    public Distance getMeasureX() {
        return Meters.of(m_x);
    }

    /**
     * Returns the Y component of the translation in a measure.
     *
     * @return The y component of the translation in a measure.
     */
    public Distance getMeasureY() {
        return Meters.of(m_y);
    }

    /**
     * Returns the Z component of the translation in a measure.
     *
     * @return The z component of the translation in a measure.
     */
    public Distance getMeasureZ() {
        return Meters.of(m_z);
    }

    /**
     * Returns a 2D translation vector representation of this translation.
     *
     * @return A 2D translation vector representation of this translation.
     */
    public Vector<N3> toVector() {
        return VecBuilder.fill(m_x, m_y, m_z);
    }

    /**
     * Returns the norm, or distance from the origin to the translation.
     *
     * @return The norm of the translation.
     */
    public double getNorm() {
        return Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    }

    /**
     * Returns the squared norm, or squared distance from the origin to the
     * translation. This is
     * equivalent to squaring the result of {@link #getNorm()}, but avoids computing
     * a square root.
     *
     * @return The squared norm of the translation.
     */
    public double getSquaredNorm() {
        return m_x * m_x + m_y * m_y + m_z * m_z;
    }

    public Translation3dDemacia rotateBy(Rotation3dDemacia other) {
        final var p = new QuaternionDemacia(0, m_x, m_y, m_z);
        final var qprime = other.getQuaternion().times(p).times(other.getQuaternion().inverse());
        m_x = qprime.getX();
        m_y = qprime.getY();
        m_z = qprime.getZ();
        return this;
    }

    public Translation3dDemacia rotateAround(Translation3dDemacia other, Rotation3dDemacia rot) {
        return minus(other).rotateBy(rot).plus(other);
    }

    public double dot(Translation3dDemacia other) {
        return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
    }

    public Vector<N3> cross(Translation3dDemacia other) {
        return VecBuilder.fill(
                m_y * other.m_z - other.m_y * m_z,
                m_z * other.m_x - other.m_z * m_x,
                m_x * other.m_y - other.m_x * m_y);
    }

    public Translation2dDemacia toTranslation2d() {
        return new Translation2dDemacia(m_x, m_y);
    }

    public Translation3dDemacia plus(Translation3dDemacia other) {
        m_x += other.m_x;
        m_y += other.m_y;
        m_z += other.m_z;
        return this;
    }

    public Translation3dDemacia minus(Translation3dDemacia other) {
        m_x -= other.m_x;
        m_y -= other.m_y;
        m_z -= other.m_z;
        return this;
    }

    public Translation3dDemacia unaryMinus() {
        m_x *= -1;
        m_y *= -1;
        m_z *= -1;
        return this;
    }

    public Translation3dDemacia times(double scalar) {
        m_x *= scalar;
        m_y *= scalar;
        m_z *= scalar;
        return this;
    }

    public Translation3dDemacia div(double scalar) {
        m_x /= scalar;
        m_y /= scalar;
        m_z /= scalar;
        return this;
    }

    public Translation3dDemacia nearest(Collection<Translation3dDemacia> translations) {
        return Collections.min(translations, Comparator.comparing(this::getDistance));
    }

    @Override
    public String toString() {
        return String.format("Translation3d(X: %.2f, Y: %.2f, Z: %.2f)", m_x, m_y, m_z);
    }

    /**
     * Checks equality between this Translation3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj instanceof Translation3dDemacia other
                && Math.abs(other.m_x - m_x) < 1E-9
                && Math.abs(other.m_y - m_y) < 1E-9
                && Math.abs(other.m_z - m_z) < 1E-9;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y, m_z);
    }

    public Translation3dDemacia interpolate(Translation3dDemacia endValue, double t) {
        m_x = MathUtil.interpolate(m_x, endValue.getX(), t);
        m_y = MathUtil.interpolate(m_y, endValue.getY(), t);
        m_z = MathUtil.interpolate(m_z, endValue.getZ(), t);
        return this;
    }

    public static final Translation3dProto proto = new Translation3dProto();
    public static final Translation3dStruct struct = new Translation3dStruct();
}

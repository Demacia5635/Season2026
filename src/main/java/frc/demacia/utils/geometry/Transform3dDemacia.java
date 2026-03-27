package frc.demacia.utils.geometry;

import java.util.Objects;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.proto.Transform3dProto;
import edu.wpi.first.math.geometry.struct.Transform3dStruct;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Transform3dDemacia implements ProtobufSerializable, StructSerializable {
    public static final Translation3dDemacia kZero = new Translation3dDemacia();

    private Translation3dDemacia m_translation;
    private Rotation3dDemacia m_rotation;

    public Transform3dDemacia(Pose3dDemacia initial, Pose3dDemacia last) {
        m_translation = last.getTranslation().minus(initial.getTranslation())
                .rotateBy(initial.getRotation().unaryMinus());

        m_rotation = last.getRotation().relativeTo(initial.getRotation());
    }

    public Transform3dDemacia(Translation3dDemacia translation, Rotation3dDemacia rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    public Transform3dDemacia(double x, double y, double z, Rotation3dDemacia rotation) {
        m_translation = new Translation3dDemacia(x, y, z);
        m_rotation = rotation;
    }

    public Transform3dDemacia(Matrix<N4, N4> matrix) {
        m_translation = new Translation3dDemacia(matrix.get(0, 3), matrix.get(1, 3), matrix.get(2, 3));
        m_rotation = new Rotation3dDemacia(matrix.block(3, 3, 0, 0));
        if (matrix.get(3, 0) != 0.0
                || matrix.get(3, 1) != 0.0
                || matrix.get(3, 2) != 0.0
                || matrix.get(3, 3) != 1.0) {
        }
    }

    public Transform3dDemacia() {
        m_translation = new Translation3dDemacia();
        m_rotation = new Rotation3dDemacia();
    }

    public Transform3dDemacia(Transform2dDemacia transform) {
        m_translation = new Translation3dDemacia(transform.getTranslation());
        m_rotation = new Rotation3dDemacia(transform.getRotation());
    }

    public Transform3dDemacia times(double scalar) {
        m_translation.times(scalar);
        m_rotation.times(scalar);
        return this;
    }

    public Transform3dDemacia div(double scalar) {
        return times(1 / scalar);
    }

    public Transform3dDemacia plus(Transform3dDemacia other) {
        var last = Pose3dDemacia.kZero.transformBy(this).transformBy(other);
        m_translation = last.getTranslation();
        m_rotation = last.getRotation().relativeTo(Rotation3dDemacia.kZero);
        return this;
    }

    public Translation3dDemacia getTranslation() {
        return m_translation;
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    public double getZ() {
        return m_translation.getZ();
    }

    /**
     * Returns the X component of the transformation's translation in a measure.
     *
     * @return The x component of the transformation's translation in a measure.
     */
    public Distance getMeasureX() {
        return m_translation.getMeasureX();
    }

    /**
     * Returns the Y component of the transformation's translation in a measure.
     *
     * @return The y component of the transformation's translation in a measure.
     */
    public Distance getMeasureY() {
        return m_translation.getMeasureY();
    }

    /**
     * Returns the Z component of the transformation's translation in a measure.
     *
     * @return The z component of the transformation's translation in a measure.
     */
    public Distance getMeasureZ() {
        return m_translation.getMeasureZ();
    }

    /**
     * Returns an affine transformation matrix representation of this
     * transformation.
     *
     * @return An affine transformation matrix representation of this
     *         transformation.
     */
    public Matrix<N4, N4> toMatrix() {
        var vec = m_translation.toVector();
        var mat = m_rotation.toMatrix();
        return MatBuilder.fill(
                Nat.N4(),
                Nat.N4(),
                mat.get(0, 0),
                mat.get(0, 1),
                mat.get(0, 2),
                vec.get(0),
                mat.get(1, 0),
                mat.get(1, 1),
                mat.get(1, 2),
                vec.get(1),
                mat.get(2, 0),
                mat.get(2, 1),
                mat.get(2, 2),
                vec.get(2),
                0.0,
                0.0,
                0.0,
                1.0);
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */
    public Rotation3dDemacia getRotation() {
        return m_rotation;
    }

    public Transform3dDemacia inverse() {
        m_translation.unaryMinus().rotateBy(m_rotation.unaryMinus());
        m_rotation.unaryMinus();
        return this;
    }

    @Override
    public String toString() {
        return String.format("Transform3d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Transform3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj instanceof Transform3dDemacia other
                && other.m_translation.equals(m_translation)
                && other.m_rotation.equals(m_rotation);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }

    /** Transform3d protobuf for serialization. */
    public static final Transform3dProto proto = new Transform3dProto();

    /** Transform3d struct for serialization. */
    public static final Transform3dStruct struct = new Transform3dStruct();
}

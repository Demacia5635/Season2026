package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.Meters;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Objects;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.proto.Pose3dProto;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.jni.Pose3dJNI;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Pose3dDemacia implements Interpolatable<Pose3dDemacia>, ProtobufSerializable, StructSerializable {
    public static final Pose3dDemacia kZero = new Pose3dDemacia();

    private Translation3dDemacia m_translation;
    private Rotation3dDemacia m_rotation;

    public Pose3dDemacia() {
        m_translation = Translation3dDemacia.kZero;
        m_rotation = Rotation3dDemacia.kZero;
    }

    public Pose3dDemacia(Translation3dDemacia translation, Rotation3dDemacia rotation) {
        this.m_translation = translation;
        this.m_rotation = rotation;
    }

    public Pose3dDemacia(double x, double y, double z, Rotation3dDemacia rotation) {
        m_translation = new Translation3dDemacia(x, y, z);
        m_rotation = rotation;
    }

    public Pose3dDemacia(Distance x, Distance y, Distance z, Rotation3dDemacia rotation) {
        this(x.in(Meters), y.in(Meters), z.in(Meters), rotation);
    }

    public Pose3dDemacia(Matrix<N4, N4> matrix) {
        m_translation = new Translation3dDemacia(matrix.get(0, 3), matrix.get(1, 3), matrix.get(2, 3));
        m_rotation = new Rotation3dDemacia(matrix.block(3, 3, 0, 0));
        if (matrix.get(3, 0) != 0
                || matrix.get(3, 1) != 0
                || matrix.get(3, 2) != 0
                || matrix.get(3, 3) != 1) {
        }
    }

    public Pose3dDemacia(Pose2dDemacia pose) {
        m_translation = new Translation3dDemacia(pose.getX(), pose.getY(), 0);
        m_rotation = new Rotation3dDemacia(0, 0, pose.getRotation().getRadians());
    }

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose. The
     * transform is applied relative to the pose's frame. Note that this differs
     * from {@link
     * Pose3d#rotateBy(Rotation3d)}, which is applied relative to the global frame
     * and around the
     * origin.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public Pose3dDemacia plus(Transform3dDemacia other) {
        return transformBy(other);
    }

    public Transform3dDemacia minus(Pose3dDemacia other) {
        final var pose = relativeTo(other);
        return new Transform3dDemacia(pose.getTranslation(), pose.getRotation());
    }

    public Translation3dDemacia getTranslation() {
        return m_translation;
    }

    public double getX() {
        return m_translation.getX();
    }

    /**
     * Returns the Y component of the pose's translation.
     *
     * @return The y component of the pose's translation.
     */
    public double getY() {
        return m_translation.getY();
    }

    /**
     * Returns the Z component of the pose's translation.
     *
     * @return The z component of the pose's translation.
     */
    public double getZ() {
        return m_translation.getZ();
    }

    /**
     * Returns the X component of the pose's translation in a measure.
     *
     * @return The x component of the pose's translation in a measure.
     */
    public Distance getMeasureX() {
        return m_translation.getMeasureX();
    }

    /**
     * Returns the Y component of the pose's translation in a measure.
     *
     * @return The y component of the pose's translation in a measure.
     */
    public Distance getMeasureY() {
        return m_translation.getMeasureY();
    }

    /**
     * Returns the Z component of the pose's translation in a measure.
     *
     * @return The z component of the pose's translation in a measure.
     */
    public Distance getMeasureZ() {
        return m_translation.getMeasureZ();
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return The rotational component of the pose.
     */
    public Rotation3dDemacia getRotation() {
        return m_rotation;
    }

    public Pose3dDemacia times(double scalar) {
        m_translation.times(scalar);
        m_rotation.times(scalar);
        return this;
    }

    public Pose3dDemacia div(double scalar) {
        return times(1 / scalar);
    }

    public Pose3dDemacia rotateBy(Rotation3dDemacia other) {
        m_translation.rotateBy(other);
        m_rotation.rotateBy(other);
        return this;
    }

    public Pose3dDemacia transformBy(Transform3dDemacia other) {
        m_translation.plus(other.getTranslation().rotateBy(m_rotation));
        m_rotation = other.getRotation().rotateBy(m_rotation);
        return this;
    }

    public Pose3dDemacia relativeTo(Pose3dDemacia other) {
        var transform = new Transform3dDemacia(other, this);
        m_translation = transform.getTranslation();
        m_rotation = transform.getRotation();
        return this;
    }

    public Pose3dDemacia rotateAround(Translation3dDemacia point, Rotation3dDemacia rot) {
        m_translation.rotateAround(point, rot);
        m_rotation.rotateBy(rot);
        return this;
    }

    public Pose3dDemacia exp(Twist3dDemacia twist) {
        var quaternion = this.getRotation().getQuaternion();
        double[] resultArray = Pose3dJNI.exp(
                this.getX(),
                this.getY(),
                this.getZ(),
                quaternion.getW(),
                quaternion.getX(),
                quaternion.getY(),
                quaternion.getZ(),
                twist.dx,
                twist.dy,
                twist.dz,
                twist.rx,
                twist.ry,
                twist.rz);
        m_translation.setX(resultArray[0]);
        m_translation.setY(resultArray[1]);
        m_translation.setZ(resultArray[2]);
        m_rotation.setQuaternion(resultArray[3], resultArray[4], resultArray[5], resultArray[6]);
        return this;
    }

    /**
     * Returns a Twist3d that maps this pose to the end pose. If c is the output of
     * {@code a.Log(b)},
     * then {@code a.Exp(c)} would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to end.
     */
    public Twist3dDemacia log(Pose3dDemacia end) {
        var thisQuaternion = this.getRotation().getQuaternion();
        var endQuaternion = end.getRotation().getQuaternion();
        double[] resultArray = Pose3dJNI.log(
                this.getX(),
                this.getY(),
                this.getZ(),
                thisQuaternion.getW(),
                thisQuaternion.getX(),
                thisQuaternion.getY(),
                thisQuaternion.getZ(),
                end.getX(),
                end.getY(),
                end.getZ(),
                endQuaternion.getW(),
                endQuaternion.getX(),
                endQuaternion.getY(),
                endQuaternion.getZ());
        return new Twist3dDemacia(
                resultArray[0],
                resultArray[1],
                resultArray[2],
                resultArray[3],
                resultArray[4],
                resultArray[5]);
    }

    /**
     * Returns an affine transformation matrix representation of this pose.
     *
     * @return An affine transformation matrix representation of this pose.
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
     * Returns a Pose2d representing this Pose3d projected into the X-Y plane.
     *
     * @return A Pose2d representing this Pose3d projected into the X-Y plane.
     */
    public Pose2dDemacia toPose2d() {
        return new Pose2dDemacia(m_translation.toTranslation2d(), m_rotation.toRotation2d());
    }

    /**
     * Returns the nearest Pose3d from a collection of poses. If two or more poses
     * in the collection
     * have the same distance from this pose, return the one with the closest
     * rotation component.
     *
     * @param poses The collection of poses to find the nearest.
     * @return The nearest Pose3d from the collection.
     */
    public Pose3dDemacia nearest(Collection<Pose3dDemacia> poses) {
        return Collections.min(
                poses,
                Comparator.comparing(
                        (Pose3dDemacia other) -> this.getTranslation().getDistance(other.getTranslation()))
                        .thenComparing(
                                (Pose3dDemacia other) -> this.getRotation().minus(other.getRotation()).getAngle()));
    }

    @Override
    public String toString() {
        return String.format("Pose3d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Pose3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj instanceof Pose3dDemacia pose
                && m_translation.equals(pose.m_translation)
                && m_rotation.equals(pose.m_rotation);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }

    @Override
    public Pose3dDemacia interpolate(Pose3dDemacia endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            var twist = log(endValue);
            var scaledTwist = new Twist3dDemacia(
                    twist.dx * t, twist.dy * t, twist.dz * t, twist.rx * t, twist.ry * t, twist.rz * t);
            return this.exp(scaledTwist);
        }
    }

    /** Pose3d protobuf for serialization. */
    public static final Pose3dProto proto = new Pose3dProto();

    /** Pose3d struct for serialization. */
    public static final Pose3dStruct struct = new Pose3dStruct();
}

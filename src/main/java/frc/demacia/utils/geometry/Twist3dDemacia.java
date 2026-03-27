package frc.demacia.utils.geometry;

import java.util.Objects;

import edu.wpi.first.math.geometry.proto.Twist3dProto;
import edu.wpi.first.math.geometry.struct.Twist3dStruct;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Twist3dDemacia implements ProtobufSerializable, StructSerializable {

    public double dx;
    public double dy;
    public double dz;
    public double rx;
    public double ry;
    public double rz;

    public Twist3dDemacia() {
    }

    public Twist3dDemacia(double dx, double dy, double dz, double rx, double ry, double rz) {
        this.dx = dx;
        this.dy = dy;
        this.dz = dz;
        this.rx = rx;
        this.ry = ry;
        this.rz = rz;
    }

    @Override
    public String toString() {
        return String.format(
                "Twist3d(dX: %.2f, dY: %.2f, dZ: %.2f, rX: %.2f, rY: %.2f, rZ: %.2f)",
                dx, dy, dz, rx, ry, rz);
    }

    /**
     * Checks equality between this Twist3d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj instanceof Twist3dDemacia other
                && Math.abs(other.dx - dx) < 1E-9
                && Math.abs(other.dy - dy) < 1E-9
                && Math.abs(other.dz - dz) < 1E-9
                && Math.abs(other.rx - rx) < 1E-9
                && Math.abs(other.ry - ry) < 1E-9
                && Math.abs(other.rz - rz) < 1E-9;
    }

    @Override
    public int hashCode() {
        return Objects.hash(dx, dy, dz, rx, ry, rz);
    }

    /** Twist3d protobuf for serialization. */
    public static final Twist3dProto proto = new Twist3dProto();

    /** Twist3d struct for serialization. */
    public static final Twist3dStruct struct = new Twist3dStruct();
}

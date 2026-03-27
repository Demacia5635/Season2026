package frc.demacia.utils.geometry;

import java.util.Objects;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.proto.Twist2dProto;
import edu.wpi.first.math.geometry.struct.Twist2dStruct;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

public class Twist2dDemacia implements ProtobufSerializable, StructSerializable {
    public double dx;
    public double dy;
    public double dtheta;

    public Twist2dDemacia() {
    }

    public Twist2dDemacia(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    @Override
    public String toString() {
        return String.format("Twist2dDemacia(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }

    @Override
    public boolean equals(Object obj) {
        return (obj instanceof Twist2dDemacia Demaciaother
                && Math.abs(Demaciaother.dx - dx) < 1E-9
                && Math.abs(Demaciaother.dy - dy) < 1E-9
                && Math.abs(Demaciaother.dtheta - dtheta) < 1E-9)
                || (obj instanceof Twist2d other
                        && Math.abs(other.dx - dx) < 1E-9
                        && Math.abs(other.dy - dy) < 1E-9
                        && Math.abs(other.dtheta - dtheta) < 1E-9);
    }

    @Override
    public int hashCode() {
        return Objects.hash(dx, dy, dtheta);
    }

    public static final Twist2dProto proto = new Twist2dProto();
    public static final Twist2dStruct struct = new Twist2dStruct();
}

package frc.robot.buttons;

import edu.wpi.first.math.Pair;

public class ButtonsConstants {

    public static final int PORT = 0;

    // button 1: 0
    // button 2: 0.1 - 0.3
    // button 3: 0.35 - 0.5
    // button 4: 0.6 - 0.75
    // button 5: 0.8 - 1

    @SuppressWarnings("unchecked")
    public static final Pair<Double, Double> [] VOLTS_RANGE = new Pair[]{
        new Pair<Double, Double>(0d, 0.5),
        new Pair<Double, Double>(0.1, 0.3),
        new Pair<Double, Double>(0.35, 0.5),
        new Pair<Double, Double>(0.55, 0.7),
        new Pair<Double, Double>(0.75, 1d)
    };
}
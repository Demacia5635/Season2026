package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Field {
    public static final double FIELD_LENGTH = 16.533114; // 54 feet
    public static final double FIELD_WIDTH = 8.069263; // 27 feet

    // Red Alliance Side Field Elements
    public final class Red{
        //Outpost
        public static final Translation2d HUMAN_PLAYER_STATION_UPPER_RIGHT = new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 1.265936);
        public static final Translation2d HUMAN_PLAYER_STATION_LOWER_LEFT = new Translation2d(FIELD_LENGTH, FIELD_WIDTH);

        //Climb
        public static final Translation2d CLIMB_UPPER_RIGHT = new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 3.298031);
        public static final Translation2d CLIMB_LOWER_LEFT = new Translation2d(FIELD_LENGTH - 1.016000, FIELD_WIDTH - 4.193381);

        //Depot
        public static final Translation2d DEPOT_UPPER_RIGHT_OUTLINE = new Translation2d(FIELD_LENGTH, 2.637631);
        public static final Translation2d DEPOT_LOWER_LEFT_OUTLINE = new Translation2d(FIELD_LENGTH - 0.685800, 1.570831);
        public static final Translation2d DEPOT_UPPER_RIGHT_INNER = new Translation2d(FIELD_LENGTH, 2.561431);
        public static final Translation2d DEPOT_LOWER_LEFT_INNER = new Translation2d(FIELD_LENGTH - 0.609600, 1.820831);

        //Alliance Line
        public static final Translation2d ALLIANCE_LINE_UPPER = new Translation2d(FIELD_LENGTH, FIELD_LENGTH - 4.003326);
        public static final Translation2d ALLIANCE_LINE_LOWER = new Translation2d(0, FIELD_LENGTH - 4.003326);

        //Upper Trench
        public static final Translation2d UPPER_TRENCH_UPPER_RIGHT = new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 4.625626);
        public static final Translation2d UPPER_TRENCH_LOWER_LEFT = new Translation2d(FIELD_LENGTH - 1.278731, FIELD_WIDTH - 4.625626);

        //Lower Trench
        public static final Translation2d LOWER_TRENCH_UPPER_RIGHT = new Translation2d(1.278731, FIELD_WIDTH - 4.625626);
        public static final Translation2d LOWER_TRENCH_LOWER_LEFT = new Translation2d(0, FIELD_WIDTH - 4.625626);

        //Upper Bump
        public static final Translation2d UPPER_BUMP_UPPER_RIGHT = new Translation2d(FIELD_LENGTH - 4.061808, FIELD_WIDTH - 1.583531);
        public static final Translation2d UPPER_BUMP_LOWER_LEFT = new Translation2d(FIELD_LENGTH - 5.186157, FIELD_WIDTH - 3.437731);

        //Lower Bump
        public static final Translation2d LOWER_BUMP_UPPER_RIGHT = new Translation2d(FIELD_LENGTH - 4.061808, 3.437731);
        public static final Translation2d LOWER_BUMP_LOWER_LEFT = new Translation2d(FIELD_LENGTH - 5.186157, 1.583531);

        //Hub
        public static final Translation2d HUB_CENTER = new Translation2d(FIELD_LENGTH - 4.6240675, FIELD_WIDTH / 2);
        public static final Translation2d HUB_UPPER_RIGHT = new Translation2d(FIELD_LENGTH - 4.039281, (FIELD_WIDTH + 1.1938) / 2);
        public static final Translation2d HUB_LOWER_LEFT = new Translation2d(FIELD_LENGTH - 4.039281, (FIELD_WIDTH - 1.1938) / 2);
    }

    // Blue Alliance Side Field Elements
    public final class Blue{
        //Outpost
        public static final Translation2d HUMAN_PLAYER_STATION_UPPER_RIGHT = new Translation2d(0, 1.265936);
        public static final Translation2d HUMAN_PLAYER_STATION_LOWER_LEFT = new Translation2d(0, 0);

        //Climb
        public static final Translation2d CLIMB_UPPER_RIGHT = new Translation2d(0, 3.298031);
        public static final Translation2d CLIMB_LOWER_LEFT = new Translation2d(1.016000, 4.193381);

        //Depot
        public static final Translation2d DEPOT_UPPER_RIGHT_OUTLINE = new Translation2d(0, FIELD_WIDTH - 2.637631);
        public static final Translation2d DEPOT_LOWER_LEFT_OUTLINE = new Translation2d(0.685800, FIELD_WIDTH - 1.570831);
        public static final Translation2d DEPOT_UPPER_RIGHT_INNER = new Translation2d(0, FIELD_WIDTH - 2.561431);
        public static final Translation2d DEPOT_LOWER_LEFT_INNER = new Translation2d(0.609600, FIELD_WIDTH - 1.820831);

        //Alliance Line
        public static final Translation2d ALLIANCE_LINE_UPPER = new Translation2d(0, 4.003326);
        public static final Translation2d ALLIANCE_LINE_LOWER = new Translation2d(FIELD_LENGTH, 4.003326);

        //Upper Trench
        public static final Translation2d UPPER_TRENCH_UPPER_RIGHT = new Translation2d(0, 4.625626);
        public static final Translation2d UPPER_TRENCH_LOWER_LEFT = new Translation2d(1.278731, 4.625626);

        //Lower Trench
        public static final Translation2d LOWER_TRENCH_UPPER_RIGHT = new Translation2d(FIELD_LENGTH - 1.278731, 4.625626);
        public static final Translation2d LOWER_TRENCH_LOWER_LEFT = new Translation2d(FIELD_LENGTH, 4.625626);

        //Upper Bump
        public static final Translation2d UPPER_BUMP_UPPER_RIGHT = new Translation2d(4.061808, 1.583531);
        public static final Translation2d UPPER_BUMP_LOWER_LEFT = new Translation2d(5.186157, 3.437731);

        //Lower Bump
        public static final Translation2d LOWER_BUMP_UPPER_RIGHT = new Translation2d(4.061808, FIELD_WIDTH - 3.437731);
        public static final Translation2d LOWER_BUMP_LOWER_LEFT = new Translation2d(5.186157, FIELD_WIDTH - 1.583531);

        //Hub
        public static final Translation2d HUB_CENTER = new Translation2d(4.6240675, FIELD_WIDTH / 2);
        public static final Translation2d HUB_UPPER_RIGHT = new Translation2d(4.039281, (FIELD_WIDTH + 1.1938) / 2);
        public static final Translation2d HUB_LOWER_LEFT = new Translation2d(4.039281, (FIELD_WIDTH - 1.1938) / 2);
    }

    //Balls Field
    public static final Translation2d BALLS_FIELD_UPPER_RIGHT = new Translation2d(FIELD_LENGTH - 7.357326, FIELD_WIDTH - 1.724431);
    public static final Translation2d BALLS_FIELD_LOWER_LEFT = new Translation2d(7.357326, 1.724431);
}

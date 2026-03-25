package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.Command;

class ButtonCommand {

    private final Command command;
    private final double minRange;
    private final double maxRange;

    public ButtonCommand(Command command, double minRange, double maxRange) {
        this.command = command;
        this.minRange = minRange;
        this.maxRange = maxRange;
    }

    public Command getCommand() {
        return command;
    }

    public boolean isInRange(double volts) {
        return volts >= minRange && volts <= maxRange;
    }
}
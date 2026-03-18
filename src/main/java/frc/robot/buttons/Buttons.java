package frc.robot.buttons;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Buttons extends SubsystemBase {

    private boolean isActive;
    private ArrayList<ButtonCommand> cmds;
    private AnalogInput sensor;
    private static Buttons m_instance;

    private Buttons() {
        isActive = false;
        sensor = new AnalogInput(ButtonsConstants.PORT);
        cmds = new ArrayList<>();
        setName("Buttons");
        SmartDashboard.putData(this);
    }

    public static Buttons getInstance() {
        if (m_instance == null) m_instance = new Buttons();
        return m_instance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("volts", sensor::getVoltage, null);
        builder.addBooleanProperty("isActive", this::isActive, this::setActive);
    }

    public void addButton(Pair<Double,Double> range, Command command) {
        cmds.add(new ButtonCommand(command, range.getFirst(), range.getSecond()));
    }

    public void addButton(double minRange, double maxRange, Command command) {
        cmds.add(new ButtonCommand(command, minRange, maxRange));
    }

    @Override
    public void periodic() {
        if (isActive) {
            for (ButtonCommand buttonCommand : cmds) {
                if (buttonCommand.isInRange(sensor.getVoltage())) { 
                    CommandScheduler.getInstance().schedule(buttonCommand.getCommand());
                }
            }
        }
    }

    public boolean isActive() {
        return isActive;
    }

    public void setActive(boolean isOn) {
        this.isActive = isOn;
    }

    public ArrayList<ButtonCommand> getCmds() {
        return cmds;
    }

    public void setCmds(ArrayList<ButtonCommand> cmds) {
        this.cmds = cmds;
    }

    public AnalogInput getSensor() {
        return sensor;
    }
}

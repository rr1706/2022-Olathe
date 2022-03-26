package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OIConstants;

public class ControllerRumble extends CommandBase {

    private Timer m_timer;
    private double m_setTime;
    private XboxController m_hid;
    private double m_strength;
    private RumbleType m_side;

    /**
     * Contructs a rumble command
     * @param hid XboxController
     * @param strength is the strength of the rumble from 0.0 to 1.0
     * @param time is how long it rumbles
     * @param side true: left rumble    false: right rumble
     */
    public ControllerRumble(XboxController hid, double strength, double time, boolean side) {
        m_hid = hid;
        m_timer = new Timer();
        m_strength = strength;
        m_setTime = time;
        if (side) {m_side = RumbleType.kLeftRumble;} else {m_side = RumbleType.kRightRumble;}
    }

    @Override
    public void initialize() {
        m_hid.setRumble(m_side, m_strength);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
      if (m_timer.hasElapsed(m_setTime) && m_setTime != 0.0) {
        m_setTime = 0.0;
        m_hid.setRumble(m_side, 0.0);
        end(false);
      }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
    }

}

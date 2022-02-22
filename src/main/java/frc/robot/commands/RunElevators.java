package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RunElevators extends CommandBase {
    private final Elevator m_low;
    private final Elevator m_high;
    
    public RunElevators(Elevator low, Elevator high){
        m_low = low;
        m_high = high;

        addRequirements(m_low, m_high);
    }

    @Override
    public void execute(){
        m_low.run();
        m_high.run();
    }

    @Override
    public void end(boolean interrupted){
        m_low.stop();
        m_high.stop();
    }
}

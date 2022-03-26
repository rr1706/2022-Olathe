package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class ReverseIntakes extends CommandBase {
    private final Intake m_rightIntake;
    private final Intake m_leftIntake;
    private final Timer m_timer = new Timer();
    private double m_ballTime = 0.00;
    private boolean m_ballDetected = false;
    public ReverseIntakes(Intake rightintake, Intake leftIntake){
        m_leftIntake = leftIntake;
        m_rightIntake = rightintake;
        addRequirements(m_leftIntake, m_rightIntake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        m_leftIntake.run(-5000);
        m_rightIntake.run(-5000);

    }

    @Override
    public void end(boolean interrupted) {
        m_leftIntake.stop();
        m_rightIntake.stop();
    }
}

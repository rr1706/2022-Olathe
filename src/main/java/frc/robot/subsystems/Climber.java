package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {

    private double m_tmpSpeed = 0.20;
    private final CANSparkMax m_motor1 = new CANSparkMax(ClimberConstants.kMotorID[0], MotorType.kBrushless);
    private final CANSparkMax m_motor2 = new CANSparkMax(ClimberConstants.kMotorID[1], MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    private final SparkMaxPIDController m_PID = m_motor1.getPIDController();
    private final DoubleSolenoid m_valve = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.kValvePorts[0], ClimberConstants.kValvePorts[1]);

    public Climber() {
        m_motor1.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_motor2.setSmartCurrentLimit(CurrentLimit.kClimber);
        m_motor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_motor2.follow(m_motor1, true);
       
        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);
        
        m_PID.setFF(0.0);
        m_PID.setOutputRange(-1.0, 1.0);
        m_PID.setP(0.00005);
        m_PID.setI(0);
        m_PID.setD(0);

        m_motor1.burnFlash();
        m_motor2.burnFlash();
        SmartDashboard.putNumber("Speed", 0);
        SmartDashboard.putBoolean("Zero Climber", false);
    }

    public void extend() {
        m_valve.set(Value.kForward);
    }

    public void retract() {
        m_valve.set(Value.kReverse);
    }

    // Change length of arm
    public void set(double pose) {
        m_PID.setReference(pose, ControlType.kPosition);
    }

    // Give motor power in terms of percent
    public void setSpeed(boolean direction) {
        SmartDashboard.putBoolean("Running Climber", true);
        if(direction) {
            m_motor1.set(m_tmpSpeed);
        } else {
            m_motor1.set(-1*m_tmpSpeed);
        }
    }

    @Override
    public void periodic() {
        double pose = m_encoder.getPosition();
        SmartDashboard.putBoolean("Running Climber", false);
        SmartDashboard.putNumber("Climber Pose", pose);
        SmartDashboard.putNumber("Current 1", m_motor1.getOutputCurrent());
        SmartDashboard.putNumber("Current 2", m_motor2.getOutputCurrent());

        if(SmartDashboard.getBoolean("Zero Climber", false)){
            m_encoder.setPosition(0.0);
            SmartDashboard.putBoolean("Zero Climber", false);
        }

        m_tmpSpeed = SmartDashboard.getNumber("Climb Speed", m_tmpSpeed);

    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
        retract();
    }
}

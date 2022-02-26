package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    private final CANSparkMax m_hoodMotor = new CANSparkMax(HoodConstants.kMotorID,MotorType.kBrushless);
    private final RelativeEncoder m_hoodEncoder = m_hoodMotor.getEncoder();
    private final SparkMaxPIDController m_hoodPID = m_hoodMotor.getPIDController();
    //private final DigitalInput m_hoodLimit = new DigitalInput(10);
    private double m_hoodAngle = 15.0;

    public ShooterHood(){
        m_hoodMotor.setSmartCurrentLimit(CurrentLimit.kHood);
        m_hoodMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        m_hoodEncoder.setPositionConversionFactor(1.2777778);
        m_hoodPID.setP(0.25);
        m_hoodPID.setI(0.0);
        m_hoodPID.setIMaxAccum(0.0003, 0);
        m_hoodPID.setD(0.0);
        m_hoodPID.setFF(0.00);
        m_hoodPID.setOutputRange(-0.33, 1.0);
        m_hoodMotor.setInverted(false);
        m_hoodMotor.burnFlash();

        SmartDashboard.putNumber("SetHoodAngle", m_hoodAngle);
        m_hoodEncoder.setPosition(0.0);
    }

    public void run() {
        m_hoodPID.setReference(m_hoodAngle, ControlType.kPosition);
    }

    public void setHood(double speed){
        m_hoodMotor.set(speed);
    }

    public void setHoodAngle(double angle){
        m_hoodAngle = angle;
        if(m_hoodAngle < 0.0){
            m_hoodAngle = 0.0;
        }
        else if(m_hoodAngle > 26.0){
            m_hoodAngle = 26.0;
        }
    }

    public void setHoodZero(){
        m_hoodEncoder.setPosition(-0.5);
        m_hoodMotor.stopMotor();
    }

    /* public boolean getHoodLimit(){
        return m_hoodLimit.get();
    } */

    public double getHoodAngle(){
        return m_hoodEncoder.getPosition();
    }

    public void stop() {
        m_hoodMotor.stopMotor();
    }

    @Override
    public void periodic() {
        setHoodAngle(SmartDashboard.getNumber("SetHoodAngle", 15.0));
        run();
    }

}

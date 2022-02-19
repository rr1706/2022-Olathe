package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

public class Rotation extends SubsystemBase {
    private final CANSparkMax m_rotMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder m_rotEncoder = m_rotMotor.getEncoder();
    private final SparkMaxPIDController m_rotPID = m_rotMotor.getPIDController();
    private double m_rotAngle = 15.0;

    public Rotation() {
        m_rotMotor.setSmartCurrentLimit(CurrentLimit.kHood);
        m_rotMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
        m_rotMotor.setIdleMode(IdleMode.kBrake);
        m_rotEncoder.setPositionConversionFactor(1.333333);
        m_rotPID.setP(0.25);
        m_rotPID.setI(0.0);
        m_rotPID.setIMaxAccum(0.0003, 0);
        m_rotPID.setD(0.0);
        m_rotPID.setFF(0.00);
        m_rotPID.setOutputRange(-0.33, 1.0);
        m_rotMotor.setInverted(false);
        m_rotMotor.burnFlash();

    }
}

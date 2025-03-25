package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.WristConstants;

public class WristSS extends SubsystemBase{

    private TalonFX m_leadWristMotor;
    private double manualVal;
    private double output;

    private double speed;
    private double setPoint;
    private double maxSpeed;

    private double p = 1.5;
    private double i;
    private double d;

    private CANcoder m_CANcoder;
    private PIDController pidController;


    public WristSS(){
        m_leadWristMotor = new TalonFX(WristConstants.WRIST_MOTOR_ID);
        m_leadWristMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_leadWristMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leadWristMotor.setInverted(true);

        m_CANcoder = new CANcoder(WristConstants.WRIST_CANCODER_ID);
        m_CANcoder.getConfigurator().apply(Robot.ctreConfigs.wristCANcoderConfig);
        m_CANcoder.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble());

        pidController = new PIDController(p, i, d);

    }

    public enum mode{
        Stop,
        SetSpeed,
        Manual, 
        PID
    }

    mode Mode = mode.Stop;

    @Override

    public void periodic() {

        switch (Mode) {
            
            case Stop:
                m_leadWristMotor.set(0);
                break;
        
            case SetSpeed:
                m_leadWristMotor.set(speed);
                break;

            case PID:
                pidController.reset();
                output = MathUtil.clamp(pidController.calculate(m_CANcoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_leadWristMotor.set(-output);
                break;

            case Manual:
                output = MathUtil.clamp(manualVal, -0.25, 0.25);
                m_leadWristMotor.set(output);
                break;
        
        }
        SmartDashboard.putNumber("Wrist Motor Speed", speed);
        SmartDashboard.putNumber("Wrist Encoder", m_CANcoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist AbsEncoder", m_CANcoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Ouput", output);
        SmartDashboard.putNumber("Wrist Setpoint", setPoint);
    }

    public void Stop(){
        Mode = mode.Stop;
    }

    public void setSpeed(double speed){
        this.speed = speed;
        Mode = mode.SetSpeed;
    }

        public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        pidController.reset();
        Mode = mode.PID;
    }

    public void Manual(double manualVal){
        this.manualVal = manualVal;
        Mode = mode.Manual;
    }
}

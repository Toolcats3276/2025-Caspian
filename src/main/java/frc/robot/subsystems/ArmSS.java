package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

public class ArmSS extends SubsystemBase{

    private TalonFX m_leadArmMotor;
    private TalonFX m_followArmMotor;

    private CANcoder m_CANCoder;
    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;

    private double speed;
    private double setPoint;
    private double maxSpeed;

    private double manualVal;
    private double output;
    private double p = 13; //16
    private double i;
    private double d = 0.005;

    public ArmSS(){
        m_leadArmMotor = new TalonFX(ArmConstants.ARM_LEAD_MOTR_ID);
        m_leadArmMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_leadArmMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leadArmMotor.setInverted(true);

        m_followArmMotor = new TalonFX(ArmConstants.ARM_FOLLOW_MOTOR_ID);
        m_followArmMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_followArmMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followArmMotor.setInverted(false);
        m_followArmMotor.setControl(new StrictFollower(m_leadArmMotor.getDeviceID()));

        m_CANCoder = new CANcoder(ArmConstants.ARM_CANCONDER_ID);
        m_CANCoder.getConfigurator().apply(Robot.ctreConfigs.armCANcoderConfig);
        m_CANCoder.setPosition(m_CANCoder.getAbsolutePosition().getValueAsDouble());

        pidController = new PIDController(p, i, d);

        profiledPIDController = new ProfiledPIDController(p, i, d,
            new TrapezoidProfile.Constraints(
                0.13, 0.13));
    }

    public enum mode{
        Stop,
        SetSpeed,
        PID,
        ManualStop,
        Manual,
    }

    mode Mode = mode.Stop;

    @Override

    public void periodic() {

        switch (Mode) {
            
            case Stop: 
                m_leadArmMotor.set(0);
                break;
        
            case SetSpeed:
                m_leadArmMotor.set(speed);
                break;

            case PID:
                pidController.reset();
                output = MathUtil.clamp(pidController.calculate(m_CANCoder.getPosition().getValueAsDouble(), setPoint), -maxSpeed, maxSpeed);
                m_leadArmMotor.set(output);
                break;
            

            case ManualStop:
                m_leadArmMotor.set(0);

            case Manual:
                output = MathUtil.clamp(manualVal, -0.25, 0.25);
                m_leadArmMotor.set(output);
                break;
        }

        SmartDashboard.putNumber("Arm Motor Speed", speed);
        SmartDashboard.putNumber("arm axis", manualVal);
        SmartDashboard.putNumber("Arm Encoder", m_CANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm AbsEncoder", m_CANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Output", output);
        SmartDashboard.putNumber("Arm Setpoint", setPoint);

        SmartDashboard.putNumber("Arm Velocity", m_CANCoder.getVelocity().getValue().in(DegreesPerSecond));
        SmartDashboard.putNumber("ProfiledPID Output", 
            profiledPIDController.calculate(m_CANCoder.getPosition().getValueAsDouble(),
                new TrapezoidProfile.State(ArmConstants.COMP, 0), 
                new TrapezoidProfile.Constraints(0.13, 0.13)));

        SmartDashboard.putNumber("PID Output", 
            pidController.calculate(m_CANCoder.getPosition().getValueAsDouble(), ArmConstants.COMP));

        SmartDashboard.putBoolean("Arm At Setpoint", atSetpoint());
    }

    public void Stop(){
        Mode = mode.Stop;
    }

    public void ManualStop(){
        Mode = mode.ManualStop;
    }

    public void Manual(double manualVal){
        this.manualVal = manualVal;
        Mode = mode.Manual;
    }

    public void PID(double setPoint, double maxSpeed){
        this.setPoint = setPoint;
        this.maxSpeed = maxSpeed;
        pidController.reset();
        Mode = mode.PID;
    }

    public double returnSetPoint(){
        return setPoint;
    }

    public void setSpeed(double speed){
        this.speed = speed;
        Mode = mode.SetSpeed;
    }

    public boolean atSetpoint(){
        return Math.abs(setPoint - m_CANCoder.getPosition().getValueAsDouble()) < 0.05;
    }
}

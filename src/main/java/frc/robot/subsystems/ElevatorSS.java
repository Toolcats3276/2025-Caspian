package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSS extends SubsystemBase{

    private TalonFX m_leadElevatorMotor;

    private Counter m_elevatorEncoder;
    private double elevatorMeasurment;

    private PIDController m_PIDController;
    private double setpoint;

    private double speed;
    private double maxSpeed;
    private double ManualVal;
    private double output;
    private double p = 0.96; //0.75, newer 0.9
    private double i = 0;
    private double d = 0.02;
    // private double p = 2.5;
    // private double i;
    // private double d = 0.0005;

    public ElevatorSS(){
        m_leadElevatorMotor = new TalonFX(ElevatorConstants.Elevator_Lead_Motor_ID);
        m_leadElevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_leadElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leadElevatorMotor.setInverted(true);

        m_elevatorEncoder = new Counter(Counter.Mode.kExternalDirection);
        m_elevatorEncoder.setUpSource(1);
        m_elevatorEncoder.setDownSource(0);
        m_elevatorEncoder.setUpSourceEdge(true, false);
        m_elevatorEncoder.setSamplesToAverage(127);
        m_elevatorEncoder.reset();

        elevatorMeasurment = m_elevatorEncoder.getDistance() / 1000;

        m_PIDController = new PIDController(p, i, d);
        m_PIDController.setTolerance(0.1);

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

        elevatorMeasurment = m_elevatorEncoder.getDistance() / 1000;

        switch (Mode) {
            
            case Stop:
                m_leadElevatorMotor.set(0);
                break;
        
            case SetSpeed:
                m_leadElevatorMotor.set(speed);
                break;

            case Manual:
                output = MathUtil.clamp(ManualVal, -0.1, 0.1);
                m_leadElevatorMotor.set(output);
                break;

            case PID:
                output = MathUtil.clamp(m_PIDController.calculate(elevatorMeasurment, setpoint), -maxSpeed, maxSpeed);
                m_leadElevatorMotor.set(-output);
                break;
            }

        SmartDashboard.putNumber("Elevator Motor Speed", output);
        SmartDashboard.putNumber("Elevator Acceleration", m_leadElevatorMotor.getAcceleration().getValue().in(DegreesPerSecondPerSecond));
        SmartDashboard.putNumber("Elevator Encoder", elevatorMeasurment);

        SmartDashboard.putBoolean("Elevator at SetPoint", atCompPose());
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    }

    public void Stop(){
        Mode = mode.Stop;
    }

    public void Manual(double ManualVal){
        this.ManualVal = ManualVal;
        Mode = mode.Manual;
    }

    public void setSpeed(double speed){
        this.speed = speed;
        Mode = mode.SetSpeed;
    }

    public void PID (double setpoint, double maxSpeed){
        this.setpoint = setpoint;
        this.maxSpeed = maxSpeed;
        Mode = mode.PID;
    }

    public void resetEncoder(){
        m_elevatorEncoder.reset();
        elevatorMeasurment = m_elevatorEncoder.getDistance();
        
    }

    public double getSetpoint(){
        return setpoint;
    }

    public boolean atCompPose(){
        if(ElevatorConstants.COMP - 3 < elevatorMeasurment && ElevatorConstants.COMP + 3.5 > elevatorMeasurment){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean atAutoCompPose(){
        if(ElevatorConstants.COMP - 3 < elevatorMeasurment && ElevatorConstants.COMP + 4.5 > elevatorMeasurment){
            return true;
        }
        else{
            return false;
        }
    }
}

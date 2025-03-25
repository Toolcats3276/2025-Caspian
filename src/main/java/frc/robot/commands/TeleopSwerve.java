package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.SwerveSS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {    
    private SwerveSS s_Swerve;  
    private ElevatorSS s_Elevator;  
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier autoAlign;

    public static PIDController LLTranslationPIDController;
    public static PIDController LLStrafePIDController;
    public static PIDController LLRotationPIDController;

    private Debouncer rotationDebouncer;
    

    public TeleopSwerve(SwerveSS s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier autoAlign, ElevatorSS s_Elevator) {
        this.s_Swerve = s_Swerve;
        this.s_Elevator = s_Elevator;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.autoAlign = autoAlign;

        LLTranslationPIDController = new PIDController(0.05, 0, 0);
            LLTranslationPIDController.reset();
        LLStrafePIDController = new PIDController(0.009, 0, 0.0);
            LLStrafePIDController.reset();
            LLStrafePIDController.setTolerance(5);

        // LLRotationPIDController = new PIDController(0.2, 0, 0.001);
        LLRotationPIDController = new PIDController(0.02, 0, 0.001);
            LLRotationPIDController.reset();
            LLRotationPIDController.setTolerance(5);
        rotationDebouncer = new Debouncer(0.25);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal;
        double strafeVal;
        double rotationVal;
        double LLTranslationVal;
        double LLStrafeVal;
        double LLRotationVal;

        if(autoAlign.getAsBoolean()){
            // LLTranslationVal = s_Swerve.m_frontLeftLL.calculateTranslationOutput_TA(0);
            // LLStrafeVal = s_Swerve.m_frontLeftLL.calculateTranslationOutput_TX(0);
            // LLRotationVal = s_Swerve.m_frontLeftLL.calculateRotationOutput(0);

            // LLTranslationVal = LLTranslationPIDController.calculate(100 - s_Swerve.m_frontLeftLL.getTA(), 80);
            LLTranslationVal = 0;

            if(s_Swerve.m_frontLeftLL.getTV()){
                LLStrafeVal = LLStrafePIDController.calculate(s_Swerve.m_frontLeftLL.getTX(), 0);
            }
            else{
                LLStrafeVal = 0;
            }
            
            if(rotationDebouncer.calculate(LLRotationPIDController.atSetpoint())){
                LLRotationVal = 0;
            }
            else if(s_Swerve.m_frontLeftLL.getTV()){
                LLRotationVal = s_Swerve.optimizedLLRotation();
            }
            else{
                LLRotationVal = 0;
            } 
        }
        else{
            LLTranslationVal = 0;
            LLStrafeVal = 0;
            LLRotationVal = 0;
        }

        if(s_Elevator.getSetpoint() > 1.2){
            double curvedTranslationVal = Math.pow(translationSup.getAsDouble(), 3)*2/3 + translationSup.getAsDouble()/3;
            double curvedStrafeVal = Math.pow(strafeSup.getAsDouble(), 3)*2/3 + strafeSup.getAsDouble()/3;
            double curvedRotationVal = Math.pow(rotationSup.getAsDouble(), 3)*2/3 + rotationSup.getAsDouble()/3;

            translationVal = curvedTranslationVal;
            strafeVal = curvedStrafeVal;
            rotationVal = curvedRotationVal;

            // translationVal = MathUtil.applyDeadband(curvedTranslationVal, Constants.stickDeadband);
            // strafeVal = MathUtil.applyDeadband(curvedStrafeVal, Constants.stickDeadband);
            // rotationVal = MathUtil.applyDeadband(curvedRotationVal, Constants.stickDeadband);
        }
        else{
            /* Get Values, Deadband*/
            translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }

        SmartDashboard.putBoolean("autoAlign", autoAlign.getAsBoolean());
        SmartDashboard.putNumber("X Trans", LLTranslationVal);
        SmartDashboard.putNumber("Y Trans", LLStrafeVal);
        SmartDashboard.putNumber("Theta Trans", LLRotationVal);

        SmartDashboard.putBoolean("Rotation Tolerance", rotationDebouncer.calculate(LLRotationPIDController.atSetpoint()));

        // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        //     translationVal = -translationVal;
        //     strafeVal = -strafeVal;
        // }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true,
            new Translation2d(LLTranslationVal, LLStrafeVal).times(Constants.Swerve.maxSpeed),
            LLRotationVal * Constants.Swerve.maxAngularVelocity
        );
    }
}
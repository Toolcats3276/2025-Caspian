package frc.robot.commands.AutoAlignmentCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSS;

public class autoAlignmentCommand extends Command {

  private SwerveSS s_Swerve;
  private BooleanSupplier alignLeft;

  public static PIDController LLTranslationPIDController;
  public static PIDController LLStrafePIDController;
  public static PIDController LLRotationPIDController;

  private Debouncer rotationDebouncer;

  public autoAlignmentCommand(SwerveSS s_Swerve, BooleanSupplier alignLeft) {
    this.s_Swerve = s_Swerve;
    this.alignLeft = alignLeft;

    addRequirements(s_Swerve);


      LLTranslationPIDController = new PIDController(0.05, 0, 0);
        LLTranslationPIDController.reset();

      LLStrafePIDController = new PIDController(0.009, 0, 0.0);
        LLStrafePIDController.reset();
        LLStrafePIDController.setTolerance(5);

      // LLRotationPIDController = new PIDController(0.2, 0, 0.001);
      LLRotationPIDController = new PIDController(0.02, 0, 0.001);
        LLRotationPIDController.reset();
        LLRotationPIDController.setTolerance(1);
        
      rotationDebouncer = new Debouncer(0.25);
  }

  @Override
  public void initialize() {
    LLStrafePIDController.reset();
  }

  @Override
  public void execute() {
    double translationVal;
    double strafeVal;
    double rotationVal;
    double LLTranslationVal;
    double LLStrafeVal;
    double LLRotationVal;

      // LLTranslationVal = s_Swerve.m_frontLeftLL.calculateTranslationOutput_TA(0);
      // LLStrafeVal = s_Swerve.m_frontLeftLL.calculateTranslationOutput_TX(0);
      // LLRotationVal = s_Swerve.m_frontLeftLL.calculateRotationOutput(0);

      // LLTranslationVal = LLTranslationPIDController.calculate(100 - s_Swerve.m_frontLeftLL.getTA(), 80);
      LLTranslationVal = 0;

      if(alignLeft.getAsBoolean()){
        if(s_Swerve.m_frontLeftLL.getTV()){
            LLStrafeVal = LLStrafePIDController.calculate(s_Swerve.m_frontLeftLL.getTX(), 0);
        }
        else{
            LLStrafeVal = 0;
        }
      }
      else{
        if(s_Swerve.m_frontRightLL.getTV()){
          LLStrafeVal = LLStrafePIDController.calculate(s_Swerve.m_frontRightLL.getTX(), 0);
        }
        else{
          LLStrafeVal = 0;
        }
      }
      
      // if(rotationDebouncer.calculate(LLRotationPIDController.atSetpoint())){
      //     LLRotationVal = 0;
      // }
      // else if(s_Swerve.m_frontLeftLL.getTV()){
      //     LLRotationVal = s_Swerve.optimizedLLRotation();
      // }
      // else{
      //     LLRotationVal = 0;
      // } 

        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true,
            new Translation2d(0, LLStrafeVal).times(Constants.Swerve.maxSpeed),
            0 * Constants.Swerve.maxAngularVelocity
        );
  }

  


  @Override
  public void end(boolean interrupted) {
    LLStrafePIDController.reset();
  }

  @Override
  public boolean isFinished() {
    return LLStrafePIDController.atSetpoint() || (!s_Swerve.m_frontLeftLL.getTV() && !s_Swerve.m_frontRightLL.getTV());
  }
}

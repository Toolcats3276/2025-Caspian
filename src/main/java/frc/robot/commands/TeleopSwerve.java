package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.SwerveSS;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("unused")
public class TeleopSwerve extends Command {  

    private SwerveSS s_Swerve;  
    private ArmSS s_Arm;
    private ElevatorSS s_Elevator;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier alignLeft;
    private BooleanSupplier alignRight;
    private BooleanSupplier top;
    private BooleanSupplier bottom;
    
    public TeleopSwerve(SwerveSS s_Swerve, ArmSS s_Arm, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier alignLeft, BooleanSupplier alignRight, BooleanSupplier top, BooleanSupplier bottom, ElevatorSS s_Elevator) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.alignLeft = alignLeft;
        this.alignRight = alignRight;
        this.top = top;
        this.bottom = bottom;
        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
    }

    @Override
    public void execute() {
        double translationVal;
        double strafeVal;
        double rotationVal;
        boolean Align_BackSide = s_Arm.returnSetPoint() == ArmConstants.L1 || s_Arm.returnSetPoint() == ArmConstants.L2 || s_Arm.returnSetPoint() == ArmConstants.L3 || s_Arm.returnSetPoint() == ArmConstants.L4;
            
        if (Align_BackSide) {
            if(alignRight.getAsBoolean()){
                if (Math.abs(SwerveSS.LLAssistantBL.getTX()) > 1.5+s_Swerve.LLStrafeOffsetBL() || Math.abs(SwerveSS.LLAssistantBL.getTY()) > 1+s_Swerve.LLTranslationOffsetBL()) {
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.TranslationCalculateBL();
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.StrafeCalculateBL();
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.HeadingCalculateBL();
                }
                else{
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
                }
            }

            else if (alignLeft.getAsBoolean()) {
                if (Math.abs(SwerveSS.LLAssistantBR.getTX()) > 1.5+s_Swerve.LLStrafeOffsetBR()|| Math.abs(SwerveSS.LLAssistantBR.getTY()) > 1+s_Swerve.LLTranslationOffsetBR()) {
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.TranslationCalculateBR();
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.StrafeCalculateBR();
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.HeadingCalculateBR();
                }
                else{
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
                }
            }

            else {
                /* Get Values, Deadband*/
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            }
        }

        else{
            if(alignLeft.getAsBoolean()){
                if (Math.abs(SwerveSS.LLAssistantFL.getTX()) > 1.5+s_Swerve.LLStrafeOffsetFL() || Math.abs(SwerveSS.LLAssistantFL.getTY()) > 1+s_Swerve.LLTranslationOffsetFL()) {
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)-s_Swerve.TranslationCalculateFL();
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)-s_Swerve.StrafeCalculateFL();
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.HeadingCalculateFL();
                }
                else{
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
                }
            }

            else if (alignRight.getAsBoolean()) {
                if (Math.abs(SwerveSS.LLAssistantFR.getTX()) > 1.5+s_Swerve.LLStrafeOffsetFR() || Math.abs(SwerveSS.LLAssistantFR.getTY()) > .25+s_Swerve.LLTranslationOffsetFR()) {
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)-s_Swerve.TranslationCalculateFR();
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)-s_Swerve.StrafeCalculateFR();
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)+s_Swerve.HeadingCalculateFR();
                }
                else{
                    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
                }
            }

            else {
                /* Get Values, Deadband*/
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            }
        }

        SmartDashboard.putBoolean("Align Left", alignLeft.getAsBoolean());
        SmartDashboard.putBoolean("Align Right", alignRight.getAsBoolean());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true, null, 0
        );
    }
}
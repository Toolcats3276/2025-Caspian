package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.LimelightAssistant;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSS extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static Pigeon2 gyro;
    public static RobotConfig config;
    public static PathPlannerPath pathfindToPath;
    
    public static PIDController LLTranslationFR;
    public static PIDController LLStrafeFR;
    public static PIDController LLTranslationFL;
    public static PIDController LLStrafeFL;
    public static PIDController LLHeadingRotationF;
    public static PIDController LLHeadingRotationB;
    public static PIDController LLTranslationBR;
    public static PIDController LLStrafeBR;
    public static PIDController LLTranslationBL;
    public static PIDController LLStrafeBL;
    public static LimelightAssistant LLAssistantFR;
    public static LimelightAssistant LLAssistantFL;
    public static LimelightAssistant LLAssistantBR;
    public static LimelightAssistant LLAssistantBL;

    public static SwerveDrivePoseEstimator m_poseEstimator;

    public Field2d LLPose;
    public Field2d BotPose;
    public boolean scoreLeft;

    private final SendableChooser<Boolean> FlipAuto;

    private NeutralModeValue driveNeutralMode;
    
    
        public SwerveSS() {
    
            driveNeutralMode = NeutralModeValue.Brake;
    
            gyro = new Pigeon2(Swerve.pigeonID);
            gyro.getConfigurator().apply(new Pigeon2Configuration());
            gyro.setYaw(0);
    
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Swerve.Mod0.constants),
                new SwerveModule(1, Swerve.Mod1.constants),
                new SwerveModule(2, Swerve.Mod2.constants),
                new SwerveModule(3, Swerve.Mod3.constants)
            };

            LLTranslationFR = new PIDController(.065, 0, 0);
            LLStrafeFR = new PIDController(0.01, 0, 0);
            LLTranslationFL = new PIDController(.027, 0, 0);
            LLStrafeFL = new PIDController(0.007, 0, 0);
            LLHeadingRotationF = new PIDController(0.014,0,0);
            
            
            LLTranslationBR = new PIDController(0.015, 0, 0);
            LLStrafeBR = new PIDController(0.007, 0, 0);
            LLTranslationBL = new PIDController(0.02, 0, 0);
            LLStrafeBL = new PIDController(0.004, 0, 0);
            LLHeadingRotationB = new PIDController(0.014,0,0);
    
            LLAssistantFR = new LimelightAssistant("limelight-fr", VecBuilder.fill(0,0,0), false);
            LLAssistantFL = new LimelightAssistant("limelight-fl", VecBuilder.fill(0,0,0), false);
            LLAssistantBR = new LimelightAssistant("limelight-br", VecBuilder.fill(0,0,0), false);
            LLAssistantBL = new LimelightAssistant("limelight-bl", VecBuilder.fill(0,0,0), false);
    
            LLPose = new Field2d();
            BotPose = new Field2d();
    
            swerveOdometry = new SwerveDriveOdometry(Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
            m_poseEstimator = new SwerveDrivePoseEstimator(
                Swerve.swerveKinematics, 
                getGyroYaw(), 
                getModulePositions(), 
                getPose(),
                LimelightConstants.STATE_STD_DEV,
                LimelightConstants.VISION_STD_DEV);
    
                FlipAuto = new SendableChooser<Boolean>();
                FlipAuto.setDefaultOption("Don't Flip", false);
                FlipAuto.addOption("Flip", true);


    
                SmartDashboard.putData(FlipAuto);
    
            try{
                config = RobotConfig.fromGUISettings();
            } 
            catch (Exception e) {
                // Handle exception as needed
                e.printStackTrace();
            }
            
            AutoBuilder.configure(
                this::getPose, 
                this::setPose, 
                this::getRobotSpeed, 
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(9, 0, 0.1), // Translation constants //3.5
                    new PIDConstants(8, 0, 0) // Rotation constants P = 1.5
                ),
                config,
                () ->  false,
                // () -> {
                //     // Boolean supplier that controls when the path will be mirrored for the red alliance
                //     // This will flip the path being followed to the red side of the field.
                //     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          
                //     var alliance = DriverStation.getAlliance();
                //     if (alliance.isPresent()) {
                //         return alliance.get() == DriverStation.Alliance.Red;
                //     }
                //     return false;
                // }, 
                this);
    
                
        }
    
        
    
    
        public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Translation2d LLTranslation, double LLRotation) {
            SwerveModuleState[] swerveModuleStates =
                Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation, 
                            getHeading()
                        )
                        : new ChassisSpeeds(
                            translation.getX(), 
                            translation.getY(), 
                            rotation)
                        );
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);
    
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        }  
        
        
        public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
            ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
            SwerveModuleState[] targetState = Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
            // SwerveDriveKinematics.desaturateWheelSpeeds(targetState, Swerve.maxSpeed);
    
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(targetState[mod.moduleNumber], false);
            }
    
          }
        // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        
        //     SwerveModuleState[] targetStates = Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        //     setModuleStates(targetStates);
        //   }
    
        /* Used by SwerveControllerCommand in Auto */
        public void setModuleStates(SwerveModuleState[] desiredStates) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);
            
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            }
        }
    
        public SwerveModuleState[] getModuleStates(){
            SwerveModuleState[] states = new SwerveModuleState[4];
            for(SwerveModule mod : mSwerveMods){
                states[mod.moduleNumber] = mod.getState();
            }
            return states;
        }
    
        public SwerveModulePosition[] getModulePositions(){
            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for(SwerveModule mod : mSwerveMods){
                positions[mod.moduleNumber] = mod.getPosition();
            }
            return positions;
        }
    
        public ChassisSpeeds getRobotSpeed(){
            return Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        }
    
        public Pose2d getPose() {
            return swerveOdometry.getPoseMeters();
        }
    
        public Pose2d getPoseEstimate(){
            return m_poseEstimator.getEstimatedPosition();
        }
    
        public void setPose(Pose2d pose) {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    
        public void resetPoseEstimate(Pose2d pose){
            m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    
        public Rotation2d getHeading(){
            return getPose().getRotation();
        }
    
        public void setHeading(Rotation2d heading){
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        }
    
        public void zeroHeading(){
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
    
        public Rotation2d getGyroYaw() {
            return Rotation2d.fromDegrees(gyro.getYaw().getValue().in(Degrees));
        }
    
        public void resetModulesToAbsolute(){
            for(SwerveModule mod : mSwerveMods){
                mod.resetToAbsolute();
            }
        }
    
    
        public void setNeutralMode(NeutralModeValue driveNeutralMode){
            this.driveNeutralMode = driveNeutralMode;
            TalonFXConfiguration swerveDriveFXConfig = Robot.ctreConfigs.swerveDriveFXConfig;
            swerveDriveFXConfig.MotorOutput.NeutralMode = driveNeutralMode;

            mSwerveMods[0].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[1].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[2].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
            mSwerveMods[3].mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
        }


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        // updatePoseEstimate();

        // LLPose.setRobotPose(getPose());
        BotPose.setRobotPose(getPose());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        
        SmartDashboard.putBoolean("FlipAutoBuilder", AutoBuilder.shouldFlip());

        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("Heading Front Right Output", HeadingCalculateFR());
        SmartDashboard.putNumber("Heading Front Left Output", HeadingCalculateFL());
        SmartDashboard.putNumber("LLRotationFR", getLLRotationAngleFR());
        SmartDashboard.putNumber("LLRotationFL", getLLRotationAngleFL());
        SmartDashboard.putNumber("Front Right translation", TranslationCalculateFR());
        SmartDashboard.putNumber("Front Left translation", TranslationCalculateFL());
        SmartDashboard.putNumber("Strafe Front Right", StrafeCalculateFR());
        SmartDashboard.putNumber("Strafe Front Left", StrafeCalculateFL());
        SmartDashboard.putNumber("Heading Back Right Output", HeadingCalculateBR());
        SmartDashboard.putNumber("Heading Back Left Output", HeadingCalculateBL());
        SmartDashboard.putNumber("LLRotationBR", getLLRotationAngleBR());
        SmartDashboard.putNumber("LLRotationBL", getLLRotationAngleBL());
        SmartDashboard.putNumber("Back Right Translation", TranslationCalculateBR());
        SmartDashboard.putNumber("Back Left translation", TranslationCalculateBL());
        SmartDashboard.putNumber("Strafe Back right", StrafeCalculateBR());
        SmartDashboard.putNumber("Strafe Back left", StrafeCalculateBL());

    }

    public double getLLRotationAngleFR(){
        double LLRotation;
        if(LLAssistantFR.getFiducialID() == 6){
            LLRotation = -60;
        }
        else if(LLAssistantFR.getFiducialID() == 7){
            LLRotation = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 8){
            LLRotation = 60;
        }
        else if(LLAssistantFR.getFiducialID() == 9){
            LLRotation = 120;
        }
        else if(LLAssistantFR.getFiducialID() == 10){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantFR.getFiducialID() == 11){
            LLRotation = -120;
        }
        else if(LLAssistantFR.getFiducialID() == 17){
            LLRotation = 60;
        }
        else if(LLAssistantFR.getFiducialID() == 18){
            LLRotation = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 19){
            LLRotation = -60;
        }
        else if(LLAssistantFR.getFiducialID() == 20){
            LLRotation = -120;
        }
        else if(LLAssistantFL.getFiducialID() == 21){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantFR.getFiducialID() == 22){
            LLRotation = 120;
        }
        else {
            LLRotation = 0;
        }

        return LLRotation;
    }
    public double LLStrafeOffsetFR(){
        double LLStrafeOffSetR;
        if(LLAssistantFR.getFiducialID() == 6){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 7){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 8){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 9){
            LLStrafeOffSetR = -0;
        }
        else if(LLAssistantFR.getFiducialID() == 10){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 11){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 17){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 18){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 19){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 20){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 22){
            LLStrafeOffSetR = 0;
        }
        else {
            LLStrafeOffSetR = 0;
        }

        return LLStrafeOffSetR;
    }
    public double LLTranslationOffsetFR(){
        double LLTranslationOffSetR;
        if(LLAssistantFR.getFiducialID() == 6){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 7){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 8){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 9){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 10){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 11){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 17){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 18){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 19){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 20){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 22){
            LLTranslationOffSetR = 0;
        }
        else {
            LLTranslationOffSetR = 0;
        }

        return LLTranslationOffSetR;
    }

    public double HeadingCalculateFR(){
        return LLHeadingRotationF.calculate(getHeading().getDegrees(), getLLRotationAngleFR());
    }

    public double StrafeCalculateFR(){
        return LLStrafeFR.calculate(LLAssistantFR.getTX(), LLStrafeOffsetFR());
    }

    public double TranslationCalculateFR(){
        return LLTranslationFR.calculate(LLAssistantFR.getTY(), LLTranslationOffsetFR());
    }








    public double getLLRotationAngleFL(){
        double LLRotation;
        if(LLAssistantFL.getFiducialID() == 6){
            LLRotation = -60;
        }
        else if(LLAssistantFL.getFiducialID() == 7){
            LLRotation = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 8){
            LLRotation = 60;
        }
        else if(LLAssistantFL.getFiducialID() == 9){
            LLRotation = 120;
        }
        else if(LLAssistantFL.getFiducialID() == 10){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantFL.getFiducialID() == 11){
            LLRotation = -120;
        }
        else if(LLAssistantFL.getFiducialID() == 17){
            LLRotation = 60;
        }
        else if(LLAssistantFL.getFiducialID() == 18){
            LLRotation = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 19){
            LLRotation = -60;
        }
        else if(LLAssistantFL.getFiducialID() == 20){
            LLRotation = -120;
        }
        else if(LLAssistantFL.getFiducialID() == 21){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantFL.getFiducialID() == 22){
            LLRotation = 120;
        }
        else {
            LLRotation = 0;
        }

        return LLRotation;
    }
    public double LLStrafeOffsetFL(){
        double LLStrafeOffSetL;
        if(LLAssistantFL.getFiducialID() == 6){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 7){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 8){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 9){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 10){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 11){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 17){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 18){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 19){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 20){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantFL.getFiducialID() == 22){
            LLStrafeOffSetL = 0;
        }
        else {
            LLStrafeOffSetL = 0;
        }

        return LLStrafeOffSetL;
    }

    public double LLTranslationOffsetFL(){
        double LLTranslationOffSetL;
        if(LLAssistantFR.getFiducialID() == 6){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 7){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 8){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 9){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 10){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 11){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 17){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 18){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 19){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 20){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantFR.getFiducialID() == 22){
            LLTranslationOffSetL = 0;
        }
        else {
            LLTranslationOffSetL = 0;
        }

        return LLTranslationOffSetL;
    }

    public double HeadingCalculateFL(){
        return LLHeadingRotationF.calculate(getHeading().getDegrees(), getLLRotationAngleFL());
    }

    public double StrafeCalculateFL(){
        return LLStrafeFL.calculate(LLAssistantFL.getTX(), LLStrafeOffsetFL());
    }

    public double TranslationCalculateFL(){
        return LLTranslationFL.calculate(LLAssistantFL.getTY(), LLTranslationOffsetFL());
    }


    public double getLLRotationAngleBR(){
        double LLRotation;
        if(LLAssistantBR.getFiducialID() == 6){
            LLRotation = 120;
        }
        else if(LLAssistantBR.getFiducialID() == 7){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantBR.getFiducialID() == 8){
            LLRotation = -120;
        }
        else if(LLAssistantBR.getFiducialID() == 9){
            LLRotation = -60;
        }
        else if(LLAssistantBR.getFiducialID() == 10){
            LLRotation = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 11){
            LLRotation = 60;
        }
        else if(LLAssistantBR.getFiducialID() == 17){
            LLRotation = 120;
        }
        else if(LLAssistantBR.getFiducialID() == 18){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantBR.getFiducialID() == 19){
            LLRotation = 120;
        }
        else if(LLAssistantBR.getFiducialID() == 20){
            LLRotation = 60;
        }
        else if(LLAssistantBR.getFiducialID() == 22){
            LLRotation = -60;
        }
        else {
            LLRotation = 0;
        }

        return LLRotation;
    }
    public double LLStrafeOffsetBR(){
        double LLStrafeOffSetR;
        if(LLAssistantBR.getFiducialID() == 6){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 7){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 8){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 9){
            LLStrafeOffSetR = -0;
        }
        else if(LLAssistantBR.getFiducialID() == 10){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 11){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 17){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 18){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 19){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 20){
            LLStrafeOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 22){
            LLStrafeOffSetR = 0;
        }
        else {
            LLStrafeOffSetR = 0;
        }

        return LLStrafeOffSetR;
    }
    public double LLTranslationOffsetBR(){
        double LLTranslationOffSetR;
        if(LLAssistantBR.getFiducialID() == 6){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 7){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 8){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 9){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 10){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 11){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 17){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 18){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 19){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 20){
            LLTranslationOffSetR = 0;
        }
        else if(LLAssistantBR.getFiducialID() == 22){
            LLTranslationOffSetR = 0;
        }
        else {
            LLTranslationOffSetR = 0;
        }

        return LLTranslationOffSetR;
    }

    public double HeadingCalculateBR(){
        return LLHeadingRotationB.calculate(getHeading().getDegrees(), getLLRotationAngleBR());
    }

    public double StrafeCalculateBR(){
        return LLStrafeBR.calculate(LLAssistantBR.getTX(), LLStrafeOffsetBR());
    }

    public double TranslationCalculateBR(){
        return LLTranslationBR.calculate(LLAssistantBR.getTY(), LLTranslationOffsetBR());
    }








    public double getLLRotationAngleBL(){
        double LLRotation;
        if(LLAssistantBL.getFiducialID() == 6){
            LLRotation = 120;
        }
        else if(LLAssistantBL.getFiducialID() == 7){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
        }
        else if(LLAssistantBL.getFiducialID() == 8){
            LLRotation = -120;
        }
        else if(LLAssistantBL.getFiducialID() == 9){
            LLRotation = -60;
        }
        else if(LLAssistantBL.getFiducialID() == 10){
            LLRotation = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 11){
            LLRotation = 60;
        }
        else if(LLAssistantBL.getFiducialID() == 17){
            LLRotation = -120;
        }
        else if(LLAssistantBL.getFiducialID() == 18){
            if (getHeading().getDegrees() < 0) {
                LLRotation = -180;
            }
            else {
                LLRotation = 180;
            }
       }
        else if(LLAssistantBL.getFiducialID() == 19){
            LLRotation = 120;
        }
        else if(LLAssistantBL.getFiducialID() == 20){
            LLRotation = 60;
        }
        else if(LLAssistantBL.getFiducialID() == 22){
            LLRotation = -60;
        }
        else {
            LLRotation = 0;
        }

        return LLRotation;
    }
    public double LLStrafeOffsetBL(){
        double LLStrafeOffSetL;
        if(LLAssistantBL.getFiducialID() == 6){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 7){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 8){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 9){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 10){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 11){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 17){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 18){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 19){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 20){
            LLStrafeOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 22){
            LLStrafeOffSetL = 0;
        }
        else {
            LLStrafeOffSetL = 0;
        }

        return LLStrafeOffSetL;
    }

    public double LLTranslationOffsetBL(){
        double LLTranslationOffSetL;
        if(LLAssistantBL.getFiducialID() == 6){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 7){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 8){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 9){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 10){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 11){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 17){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 18){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 19){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 20){
            LLTranslationOffSetL = 0;
        }
        else if(LLAssistantBL.getFiducialID() == 22){
            LLTranslationOffSetL = 0;
        }
        else {
            LLTranslationOffSetL = 0;
        }

        return LLTranslationOffSetL;
    }

    public double HeadingCalculateBL(){
        return LLHeadingRotationB.calculate(getHeading().getDegrees(), getLLRotationAngleBL());
    }

    public double StrafeCalculateBL(){
        return LLStrafeBL.calculate(LLAssistantBL.getTX(), LLStrafeOffsetBL());
    }

    public double TranslationCalculateBL(){
        return LLTranslationBL.calculate(LLAssistantBL.getTY(), LLTranslationOffsetBL());
    }

    // public void updatePoseEstimate(){
    //     m_poseEstimator.update(
    //         getGyroYaw(), 
    //         getModulePositions());

    //     m_frontLeftLL.updatePoseEstimates();
    //     m_frontRightLL.updatePoseEstimates();
    //     m_backLeftLL.updatePoseEstimates();
    //     m_backRightLL.updatePoseEstimates();

    //     if(!m_frontLeftLL.rejectUpdate()){
    //         m_poseEstimator.setVisionMeasurementStdDevs(m_frontLeftLL.proportionalVisionStdDevs());
    //         m_poseEstimator.addVisionMeasurement(m_frontLeftLL.getPoseEstimate(), m_frontLeftLL.getTimestamp());
    //     }

    //     if(!m_frontRightLL.rejectUpdate()){
    //         m_poseEstimator.setVisionMeasurementStdDevs(m_frontRightLL.proportionalVisionStdDevs());
    //         m_poseEstimator.addVisionMeasurement(m_frontRightLL.getPoseEstimate(), m_frontRightLL.getTimestamp());
    //     }

    //     if(!m_backLeftLL.rejectUpdate()){
    //         m_poseEstimator.setVisionMeasurementStdDevs(m_backLeftLL.proportionalVisionStdDevs());
    //         m_poseEstimator.addVisionMeasurement(m_backLeftLL.getPoseEstimate(), m_backLeftLL.getTimestamp());
    //     }

    //     if(!m_backRightLL.rejectUpdate()){
    //         m_poseEstimator.setVisionMeasurementStdDevs(m_backRightLL.proportionalVisionStdDevs());
    //         m_poseEstimator.addVisionMeasurement(m_backRightLL.getPoseEstimate(), m_backRightLL.getTimestamp());
    //     }
    // }

    // public void setPathfindingPath() throws FileVersionException, IOException, org.json.simple.parser.ParseException{
    //     var FID = m_frontLeftLL.getFiducialID();

    //     if(alignLeft.getAsBoolean()){
    //         // left side paths
    //         switch((int)FID){
    //             case 6: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 7: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 8: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 9: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 10: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 11: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 17: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 18: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 19: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 20: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 21: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 22: pathfindToPath = PathPlannerPath.fromPathFile("Test");

    //             default: 
    //             if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
    //                 pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //                 System.out.println("Invalid Tag ID " + FID);
    //             }
    //             else {
    //                 pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //                 System.out.println("Invalid Tag ID " + FID);
    //             }
    //         }
    //     }
    //     else{
    //         // right side paths
    //         switch((int)FID){
    //             case 6: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 7: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 8: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 9: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 10: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 11: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 17: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 18: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 19: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 20: pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //             case 21: pathfindToPath = PathPlannerPath.fromPathFile("Test");
                
    //             default: 
    //             if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
    //                 pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //                 System.out.println("Invalid Tag ID " + FID);
    //             }
    //             else {
    //                 pathfindToPath = PathPlannerPath.fromPathFile("Test");
    //                 System.out.println("Invalid Tag ID " + FID);
    //             }
    //         }
    //     }
    // }

    // public PathPlannerPath getPathfindingPath(BooleanSupplier alignLeft){
    //     this.alignLeft = alignLeft;
    //     return pathfindToPath;
    // }



    // public double getXPoseEstimateError(){
    //     return Math.abs(m_poseEstimator.getEstimatedPosition().getX() - swerveOdometry.getPoseMeters().getX());
    // }
    // public double getYPoseEstimateError(){
    //     return Math.abs(m_poseEstimator.getEstimatedPosition().getY() - swerveOdometry.getPoseMeters().getY());
    // }
    // public double getThetaPoseEstimateError(){
    //     return Math.abs(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() - swerveOdometry.getPoseMeters().getRotation().getDegrees());
    // }


    // public Transform2d getTransformationToReef(){
    //     double tagID;
    //     Transform2d robotTransform2d;
    //     // Pose2d rightPose2D = m_RightLimelight.getPoseEstimate();
        
    //     // robotTransform2d = LimelightConstants.TAG_6_L_POSE2D.minus(getPoseEstimate());

    //     // if (scoreLeft == true){
    //     //     if(m_frontLeftLL.getTV()){
    //     //         tagID = m_frontLeftLL.getFiducialID();
    //     //         if(tagID == 6){
    //     //             robotTransform2d = getPoseEstimate().minus(LimelightConstants.TAG_6_L_POSE2D);
    //     //         }
    //     //         else if(tagID == 17){
    //     //             robotTransform2d = getPoseEstimate().minus(LimelightConstants.TAG_17_L_POSE2D);
    //     //         }
    //     //         else{
    //     //             robotTransform2d = new Transform2d();
    //     //         }
    //     //     }
    //     //     else{
    //     //         robotTransform2d = new Transform2d();
    //     //     }
    //     // }
    //     // else{
    //     //     robotTransform2d = new Transform2d();
    //     // }


    //     // if (scoreLeft == false){
    //     //     if(m_RightLimelight.getTV()){
    //     //         tagID = m_RightLimelight.getFiducialID();
    //     //         if(tagID == 6){
    //     //             robotTransform2d = rightPose2D.minus(LimelightConstants.TAG_6_R_POSE2D);
    //     //         }
    //     //         else if(tagID == 17){
    //     //             robotTransform2d = rightPose2D.minus(LimelightConstants.TAG_17_R_POSE2D);
    //     //         }
    //     //         else{
    //     //             robotTransform2d = new Transform2d();
    //     //         }
    //     //     }
    //     //     else{
    //     //         robotTransform2d = new Transform2d();
    //     //     }
    //     // }
    //     // else{
    //     //     robotTransform2d = new Transform2d();
    //     // }

    //     return robotTransform2d;

    // }

}
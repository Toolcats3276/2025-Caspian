package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.vision.LimelightAssistant;
import frc.robot.Constants.Swerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
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
    public BooleanSupplier alignLeft;

    
    public final LimelightAssistant m_frontLeftLL;
    public final LimelightAssistant m_frontRightLL;
    public final LimelightAssistant m_backLeftLL;
    public final LimelightAssistant m_backRightLL;

    public static SwerveDrivePoseEstimator m_poseEstimator;

    public Field2d LLPose;
    public Field2d BotPose;
    public boolean scoreLeft;

    private final SendableChooser<Boolean> FlipAuto;


    public SwerveSS() {
        gyro = new Pigeon2(Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Swerve.Mod0.constants),
            new SwerveModule(1, Swerve.Mod1.constants),
            new SwerveModule(2, Swerve.Mod2.constants),
            new SwerveModule(3, Swerve.Mod3.constants)
        };

        m_frontLeftLL = new LimelightAssistant("limelight-fl", LimelightConstants.VISION_STD_DEV, true);
        m_frontRightLL = new LimelightAssistant("limelight-fr", LimelightConstants.VISION_STD_DEV, true);
        m_backLeftLL = new LimelightAssistant("limelight-bl", LimelightConstants.VISION_STD_DEV, true);
        m_backRightLL = new LimelightAssistant("limelight-br", LimelightConstants.VISION_STD_DEV, true);

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
                                ).plus(new ChassisSpeeds(
                                    LLTranslation.getX(),
                                    LLTranslation.getY(),
                                    LLRotation
                                ))

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

        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        SmartDashboard.putNumber("LLRotationAngle", getLLRotationAngle());

        SmartDashboard.putData("LL Pose", LLPose);
        SmartDashboard.putData("Bot Pose", BotPose);

        SmartDashboard.putNumber("X Chassis Speed", getRobotSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("Y Chassis Speed", getRobotSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("Omega Chassis Speed", getRobotSpeed().omegaRadiansPerSecond);

        
        SmartDashboard.putBoolean("FlipAutoBuilder", AutoBuilder.shouldFlip());

    }

    public void updatePoseEstimate(){
        m_poseEstimator.update(
            getGyroYaw(), 
            getModulePositions());

        m_frontLeftLL.updatePoseEstimates();
        m_frontRightLL.updatePoseEstimates();
        m_backLeftLL.updatePoseEstimates();
        m_backRightLL.updatePoseEstimates();

        if(!m_frontLeftLL.rejectUpdate()){
            m_poseEstimator.setVisionMeasurementStdDevs(m_frontLeftLL.proportionalVisionStdDevs());
            m_poseEstimator.addVisionMeasurement(m_frontLeftLL.getPoseEstimate(), m_frontLeftLL.getTimestamp());
        }

        if(!m_frontRightLL.rejectUpdate()){
            m_poseEstimator.setVisionMeasurementStdDevs(m_frontRightLL.proportionalVisionStdDevs());
            m_poseEstimator.addVisionMeasurement(m_frontRightLL.getPoseEstimate(), m_frontRightLL.getTimestamp());
        }

        if(!m_backLeftLL.rejectUpdate()){
            m_poseEstimator.setVisionMeasurementStdDevs(m_backLeftLL.proportionalVisionStdDevs());
            m_poseEstimator.addVisionMeasurement(m_backLeftLL.getPoseEstimate(), m_backLeftLL.getTimestamp());
        }

        if(!m_backRightLL.rejectUpdate()){
            m_poseEstimator.setVisionMeasurementStdDevs(m_backRightLL.proportionalVisionStdDevs());
            m_poseEstimator.addVisionMeasurement(m_backRightLL.getPoseEstimate(), m_backRightLL.getTimestamp());
        }
    }

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

    public double getLLRotationAngle(){
        double LLRotation;
        if(m_frontLeftLL.getFiducialID() == 6){
            LLRotation = -60;
        }
        else if(m_frontLeftLL.getFiducialID() == 7){
            LLRotation = 0;
        }
        else if(m_frontLeftLL.getFiducialID() == 8){
            LLRotation = 60;
        }
        else if(m_frontLeftLL.getFiducialID() == 9){
            LLRotation = 120;
        }
        else if(m_frontLeftLL.getFiducialID() == 10){
            LLRotation = 180;
        }
        else if(m_frontLeftLL.getFiducialID() == 11){
            LLRotation = -120;
        }
        else if(m_frontLeftLL.getFiducialID() == 17){
            LLRotation = -60;
        }
        else if(m_frontLeftLL.getFiducialID() == 18){
            LLRotation = 0;
        }
        else if(m_frontLeftLL.getFiducialID() == 19){
            LLRotation = 60;
        }
        else if(m_frontLeftLL.getFiducialID() == 20){
            LLRotation = 120;
        }
        else if(m_frontLeftLL.getFiducialID() == 21){
            LLRotation = 180;
        }
        else if(m_frontLeftLL.getFiducialID() == 22){
            LLRotation = -120;
        }
        else {
            LLRotation = 0;
        }

        return LLRotation;
    }


    public double optimizedLLRotation() {
        var delta = getLLRotationAngle() - getHeading().getDegrees();
        if (Math.abs(delta) > 90.0) {
            return -TeleopSwerve.LLRotationPIDController.calculate(getHeading().getDegrees(), getLLRotationAngle());
        } 
        else {
            return TeleopSwerve.LLRotationPIDController.calculate(getHeading().getDegrees(), getLLRotationAngle());
        }
    }







    public double getXPoseEstimateError(){
        return Math.abs(m_poseEstimator.getEstimatedPosition().getX() - swerveOdometry.getPoseMeters().getX());
    }
    public double getYPoseEstimateError(){
        return Math.abs(m_poseEstimator.getEstimatedPosition().getY() - swerveOdometry.getPoseMeters().getY());
    }
    public double getThetaPoseEstimateError(){
        return Math.abs(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() - swerveOdometry.getPoseMeters().getRotation().getDegrees());
    }



    public void setScoringSide(String scoreingSide){
        if(scoreingSide == "left"){
            scoreLeft = true;
        }
        else{
            scoreLeft = false;
        }
    }

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
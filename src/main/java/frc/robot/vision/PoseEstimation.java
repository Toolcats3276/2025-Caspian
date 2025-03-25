// package frc.robot.vision;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.LimelightConstants;
// import frc.robot.subsystems.SwerveSS;

// public class PoseEstimation extends SubsystemBase{

//     private SwerveSS s_Swerve;
//     // private LimelightAssistant m_RightLimelight;
//     private LimelightAssistant m_LeftLimelight;

//     public static SwerveDrivePoseEstimator m_poseEstimator;
//     public static Field2d m_poseEstimation;

//     // public PoseEstimation(SwerveSS s_Swerve, LimelightAssistant m_RightLimelight, LimelightAssistant m_LeftLimelight){
//     public PoseEstimation(SwerveSS s_Swerve, LimelightAssistant m_LeftLimelight){
//         this.s_Swerve = s_Swerve;
//         // this.m_RightLimelight = m_RightLimelight;
//         this.m_LeftLimelight = m_LeftLimelight;

//         m_poseEstimator = new SwerveDrivePoseEstimator(
//             Constants.Swerve.swerveKinematics,
//             s_Swerve.getGyroYaw(),
//             s_Swerve.getModulePositions(),
//             new Pose2d(),
//             VecBuilder.fill(99999, 99999, Units.degreesToRadians(5)),
//             VecBuilder.fill(0.000005, 0.000005, Units.degreesToRadians(30)));

//     }

//     public void updatePoseEstimate(){
//         m_poseEstimator.update(
//             s_Swerve.getGyroYaw(), 
//             s_Swerve.getModulePositions());

//             m_LeftLimelight.updatePoseEstimates();
//             // m_RightLimelight.updatePoseEstimates();

//             if(!m_LeftLimelight.rejectUpdate()){
//                 m_poseEstimator.setVisionMeasurementStdDevs(m_LeftLimelight.getVisionMeasurementStdDevs());
//                 m_poseEstimator.addVisionMeasurement(m_LeftLimelight.getPoseEstimate(), m_LeftLimelight.getTimestamp());
//             }
//             // if(!m_RightLimelight.rejectUpdate()){
//             //     m_poseEstimator.setVisionMeasurementStdDevs(m_RightLimelight.getVisionMeasurementStdDevs());
//             //     m_poseEstimator.addVisionMeasurement(m_RightLimelight.getPoseEstimate(), m_RightLimelight.getTimestamp());
//             // }
//     }

//     private void resetPosition(){
//         if(m_LeftLimelight.getTV()){
//             m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), m_LeftLimelight.getPoseEstimate());
//         }
//         // if(m_RightLimelight.getTV()){
//         //     m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), m_RightLimelight.getPoseEstimate());
//         // }
//     }

//     @Override
//     public void periodic() {
//         updatePoseEstimate();
//         resetPosition();
//         SmartDashboard.putData("Pose Estimate", m_poseEstimation);
//     }

// }
// package frc.robot.vision;

// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LimelightConstants;

// public class ReefAlignment extends SubsystemBase{

//     private boolean scoreLeft;
//     // private LimelightAssistant m_RightLimelight;
//     private LimelightAssistant m_LeftLimelight;
//     private PoseEstimation m_PoseEstimation;
//     private Field2d fieldPose;
    
//     // public ReefAlignment(LimelightAssistant m_RightLimelight, LimelightAssistant m_LeftLimelight){
//     public ReefAlignment(LimelightAssistant m_LeftLimelight, PoseEstimation s_PoseEstimation){
//         scoreLeft = true;
//         // this.m_RightLimelight = m_RightLimelight;
//         this.m_LeftLimelight = m_LeftLimelight;
//         this.m_PoseEstimation = m_PoseEstimation;
//         fieldPose = new Field2d();
//     }

//     public void setScoringSide(String scoreingSide){
//         if(scoreingSide == "left"){
//             scoreLeft = true;
//         }
//         else{
//             scoreLeft = false;
//         }
//     }

//     public Transform2d getTransformationToReef(){
//         double tagID;
//         Transform2d robotTransform2d;
//         // Pose2d rightPose2D = m_RightLimelight.getPoseEstimate();
//         Pose2d leftPose2D = m_LeftLimelight.getPoseEstimate();
        
//         if (scoreLeft == true){
//             if(m_LeftLimelight.getTV()){
//                 tagID = m_LeftLimelight.getFiducialID();
//                 if(tagID == 6){
//                     robotTransform2d = leftPose2D.minus(LimelightConstants.TAG_6_L_POSE2D);
//                 }
//                 else if(tagID == 17){
//                     robotTransform2d = leftPose2D.minus(LimelightConstants.TAG_17_L_POSE2D);
//                 }
//                 else{
//                     robotTransform2d = new Transform2d();
//                 }
//             }
//             else{
//                 robotTransform2d = new Transform2d();
//             }
//         }
//         else{
//             robotTransform2d = new Transform2d();
//         }


//         // if (scoreLeft == false){
//         //     if(m_RightLimelight.getTV()){
//         //         tagID = m_RightLimelight.getFiducialID();
//         //         if(tagID == 6){
//         //             robotTransform2d = rightPose2D.minus(LimelightConstants.TAG_6_R_POSE2D);
//         //         }
//         //         else if(tagID == 17){
//         //             robotTransform2d = rightPose2D.minus(LimelightConstants.TAG_17_R_POSE2D);
//         //         }
//         //         else{
//         //             robotTransform2d = new Transform2d();
//         //         }
//         //     }
//         //     else{
//         //         robotTransform2d = new Transform2d();
//         //     }
//         // }
//         // else{
//         //     robotTransform2d = new Transform2d();
//         // }

//         return robotTransform2d.inverse();

//     }


//     @Override
//     public void periodic(){

//         m_LeftLimelight.updatePoseEstimates();
//         fieldPose.setRobotPose(m_LeftLimelight.getPoseEstimate());

//         SmartDashboard.putNumber("X Trans", getTransformationToReef().getX());
//         SmartDashboard.putNumber("Y Trans", getTransformationToReef().getY());
//         SmartDashboard.putNumber("Theta Trans", getTransformationToReef().getRotation().getRotations());

//         SmartDashboard.putBoolean("Tag Visable", m_LeftLimelight.getTV());
//         SmartDashboard.putNumber("Tag ID", m_LeftLimelight.getFiducialID());

//         getTransformationToReef();
//     }

    
// }

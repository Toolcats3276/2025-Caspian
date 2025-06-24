package frc.robot.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSS;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class LimelightAssistant {

    private String limelightName;
    private Matrix<N3,N1> visionMeasurementStdDevs;
    private boolean useMegaTag1;

    // private boolean doRejectUpdate;

    // LimelightHelpers.PoseEstimate megaTag1PoseEstimator;
    // private Pose2d megaTag1Pose2d;
    // private double megaTag1Timestamp;

    // LimelightHelpers.PoseEstimate megaTag2PoseEstimator;
    // private Pose2d megaTag2Pose2d;
    // private double megaTag2Timestamp;


    private final PIDController rotationPidController;
    private final PIDController translationPidController;


    
    /**
   * Constructs a LimelightUpdater to quickly add multiple limelights to the robot for tag allignment and pose estimation.
   *
   * @param limelightName The name of the limelight as set in the WebUI.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less (can be changed later).
   * @param useMegaTag1 This will give mega tag 1 estimates if true and mega tag 2 estimates if false (can be changed later).
   */

    public LimelightAssistant(String limelightName,Matrix<N3,N1> visionMeasurementStdDevs, boolean useMegaTag1){
        
        this.limelightName = limelightName;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        this.useMegaTag1 = useMegaTag1;

        // doRejectUpdate = false;


        // megaTag1PoseEstimator = new PoseEstimate();
        // megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        // megaTag1Pose2d = new Pose2d();

        // megaTag2PoseEstimator = new PoseEstimate();
        // megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        // megaTag2Pose2d = new Pose2d();

        rotationPidController = new PIDController(0.006, 0, 0);
        translationPidController = new PIDController(0.001, 0, 0);

    }

    /**
     * @return Horizontal Offset From Crosshair To Target.
     */
    public double getTX(){
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * @return Vertical Offset From Crosshair To Target.
     */
    public double getTY(){
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * @return Target Area (0% of image to 100% of image).
     */
    public double getTA(){
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * @return True if valid target exists. False if no valid targets exist.
     */
    public boolean getTV(){
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * @return The Fiducial tag ID that is being targeted.
     */
    public double getFiducialID(){
        return LimelightHelpers.getFiducialID(limelightName);
    }


    /**
     * Method to calculate the rotational output for drivetrain aiming using TX as a measurment.
     * Needs PID Constants tuned in Limelight Constants.
     * @param setPoint The desired point to aim at.
     * @return The output for drivetrain aiming
     */
    public double calculateRotationOutput(double setPoint){
        double output;

        if(getTV()){
        output = rotationPidController.calculate(getTX(), setPoint);
        }
        else{
            output = 0;
        }

        return output;
    }

    /**
     * Method to calculate the translational output for drivetrain ranging using TY as a measurment.
     * Needs PID Constants tuned in Limelight Constants.
     * @param setPoint The desired point to range to.
     * @return The output for drivetrain ranging
     */
    public double calculateTranslationOutput_TY(double setPoint){
        double output;

        if(getTV()){
        output = translationPidController.calculate(getTY(), setPoint);
        }
        else{
            output = 0;
        }

        return output;
    }

    /**
     * Method to calculate the translational output for drivetrain ranging using TX as a measurment.
     * Needs PID Constants tuned in Limelight Constants.
     * @param setPoint The desired point to range to.
     * @return The output for drivetrain ranging
     */
    public double calculateTranslationOutput_TX(double setPoint){
        double output;

        if(getTV()){
        output = translationPidController.calculate(getTX(), setPoint);
        }
        else{
            output = 0;
        }

        return output;
    }

    /**
     * Method to calculate the translational output for drivetrain ranging using TA as a measurment.
     * Needs PID Constants tuned in Limelight Constants.
     * @param setPoint The desired point to range to.
     * @return The output for drivetrain ranging
     */
    public double calculateTranslationOutput_TA(double setPoint){
        double output;

        if(getTV()){
        output = translationPidController.calculate(100 - getTA(), setPoint);
        }
        else{
            output = 0;
        }

        return output;
    }


    /**
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     * in meters, y position in meters, and heading in radians). Increase these numbers to trust
     * the vision pose measurement less (can be changed later).
     */
    public void setVisionMeasurementStdDevs(Matrix<N3,N1> visionMeasurementStdDevs){
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    /** 
     * @return The standard deviations of the vision pose measurments (x position
     * in meters, y position in meters, and heading in radians). Increase these numbers to trust
     * the vision pose measurement less (can be changed later).
    */
    public Matrix<N3,N1> getVisionMeasurementStdDevs(){
        return visionMeasurementStdDevs;
    }

    /**
     * @param useMegaTag1 This will change weather or not to give mega tag 1 estimates if true and mega tag 2 estimates if false.
     */
    public void setMegaTagMode(boolean useMegaTag1){
        this.useMegaTag1 = useMegaTag1;
    }

    /**
     * @return This will give mega tag 1 estimates if true and mega tag 2 estimates if false.
     */
    public boolean getMegaTagMode(){
        return useMegaTag1;
    }

    /**
     * @return weather or not limelight pose should be updated
     */
    // public boolean rejectUpdate(){

    //     if(useMegaTag1 == true)
    //     {
    //         if(megaTag1PoseEstimator != null){

    //         LimelightHelpers.PoseEstimate megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    //           if(megaTag1PoseEstimator.tagCount == 1 & megaTag1PoseEstimator.rawFiducials.length == 1)
    //           {
    //             if(megaTag1PoseEstimator.rawFiducials[0].ambiguity > .7)
    //             {
    //               doRejectUpdate = true;
    //             }
    //             if(megaTag1PoseEstimator.rawFiducials[0].distToCamera > 3)
    //             {
    //               doRejectUpdate = true;
    //             }
    //           }
    //           if(megaTag1PoseEstimator.tagCount == 0)
    //           {
    //             doRejectUpdate = true;
    //           }
    //         }
    //     }

    //     else if (useMegaTag1 == false & megaTag2PoseEstimator != null)
    //     {
            
    //     LimelightHelpers.SetRobotOrientation(limelightName, SwerveSS.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //     LimelightHelpers.PoseEstimate megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    //       if(Math.abs(SwerveSS.gyro.getAngularVelocityYWorld().getValue().in(DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //       {
    //         doRejectUpdate = true;
    //       }
    //       if(megaTag2PoseEstimator.tagCount == 0)
    //       {
    //         doRejectUpdate = true;
    //       }
    //     }
        
    //     else{
    //         doRejectUpdate = false;
    //     }
    
    //     return doRejectUpdate;
    // }

    // /**
    //  * Updates mega tag 1 and 2 variables with vision measurments. 
    //  * This should be called every loop.
    //  */
    // public void updatePoseEstimates(){
    //     megaTag1PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    //     megaTag2PoseEstimator = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //     updatePoseEstimateVariables();

        
    //     publishSmartDashboard();
    // }

    // /**
    //  * Updates mega tag 1 and 2 Pose2d and Timestamp variables
    //  */
    // private void updatePoseEstimateVariables(){
    //     if(!rejectUpdate()){
    //         if(useMegaTag1 & megaTag1PoseEstimator != null){
    //             megaTag1Pose2d = megaTag1PoseEstimator.pose;
    //             megaTag1Timestamp = megaTag1PoseEstimator.timestampSeconds;
    //         }
    //         else if(!useMegaTag1 & megaTag2PoseEstimator != null){
    //             megaTag2Pose2d = megaTag2PoseEstimator.pose;
    //             megaTag2Timestamp = megaTag2PoseEstimator.timestampSeconds;
    //         }
    //     }
    // }

    // /**
    //  * @return The vision pose measurments taken from the limelight.
    //  */
    // public Pose2d getPoseEstimate(){
    //     if(useMegaTag1 == true){
    //         return megaTag1Pose2d;
    //     }
    //     else{
    //         return megaTag2Pose2d;
    //     }
    // }

    // /**
    //  * @return The time stamp taken from the limelight.
    //  */
    // public double getTimestamp(){
    //     if(useMegaTag1 == true){
    //         return megaTag1Timestamp;
    //     }
    //     else{
    //         return megaTag2Timestamp;
    //     }
    // }

    public double getDistance(){
        var distanceFromPrimaryTag =
        LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getTranslation().getDistance(LimelightHelpers.getBotPose3d(limelightName).getTranslation());
        return distanceFromPrimaryTag;
    }

    public Matrix<N3, N1> proportionalVisionStdDevs() {
        var translationStdDev = LimelightConstants.translationStdDevCoefficient * Math.pow(getDistance(), 2);
        // var rotationStdDev = LimelightConstants.rotationStdDevCoefficient * Math.pow(getDistance(), 2);

        return VecBuilder.fill(translationStdDev, translationStdDev, 9999999);
    }


    public void publishSmartDashboard(){
        SmartDashboard.putNumber(limelightName + " TX", getTX());
        SmartDashboard.putNumber(limelightName + " TY", getTY());
        SmartDashboard.putNumber(limelightName + " TA", getTA());
        SmartDashboard.putBoolean(limelightName + " TV", getTV());
        SmartDashboard.putNumber(limelightName + " Tag ID", getFiducialID());
    }


    /**
     * Method to force Limelight LEDs on
     */
    public void forceOn(){
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    /**
     * Method to force Limelight LEDs off
     */
    public void forceOff(){
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }

    /**
     * Method to force Limelight LEDs to blink
     */
    public void forceBlink(){
        LimelightHelpers.setLEDMode_ForceBlink(limelightName);
    }

}

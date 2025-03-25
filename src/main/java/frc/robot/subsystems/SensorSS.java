package frc.robot.subsystems;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class SensorSS extends SubsystemBase{

    private LaserCan m_algaeLaserCAN;

    private Debouncer algaeDebouncer;
    private Debouncer algaeInfeedDelay;

    private LaserCan m_coralLaserCAN;

    private LaserCan m_leftReefLaserCAN;
    private LaserCan m_rightReefLaserCAN;

    LaserCan.Measurement leftReefMeasurement;
    LaserCan.Measurement rightReefMeasurement;

    private Debouncer coralDebouncer;
    private Debouncer coralSourceInfeedDelay;

    LaserCan.Measurement coralLaserMeasurement;
    LaserCan.Measurement algaeLaserMeasurment;

    private boolean endCommand = false;

    private boolean infeedState;
    
    private String algaeInfeedState = InfeedConstants.ALGAE_INFEED_GROUND;
    private int algaeInfeedInt = 1;
    private boolean L4State;
        

    public SensorSS(){
        m_algaeLaserCAN = new LaserCan(2);
        try {
            m_algaeLaserCAN.setRegionOfInterest(new RegionOfInterest(8, 8, 4, 4));
        } catch (ConfigurationFailedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        try {
            m_algaeLaserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        } catch (ConfigurationFailedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        algaeLaserMeasurment = m_algaeLaserCAN.getMeasurement();

        algaeDebouncer = new Debouncer(0.1);
        algaeInfeedDelay = new Debouncer(3);
        // algaeInfeedL1Delay = new Debouncer(3);

        m_coralLaserCAN = new LaserCan(1);
        coralLaserMeasurement = m_coralLaserCAN.getMeasurement();

        coralDebouncer = new Debouncer(0.1);
        coralSourceInfeedDelay = new Debouncer(0.5);


        m_leftReefLaserCAN = new LaserCan(3);
        m_rightReefLaserCAN = new LaserCan(4);

        leftReefMeasurement = m_leftReefLaserCAN.getMeasurement();
        rightReefMeasurement = m_rightReefLaserCAN.getMeasurement();
        
    }

    @Override
    public void periodic() {
            algaeLaserMeasurment = m_algaeLaserCAN.getMeasurement();
            coralLaserMeasurement = m_coralLaserCAN.getMeasurement();

            leftReefMeasurement = m_leftReefLaserCAN.getMeasurement();
            rightReefMeasurement = m_rightReefLaserCAN.getMeasurement();
    

        if(algaeLaserMeasurment != null){
            SmartDashboard.putNumber("algaeMeasurement", algaeLaserMeasurment.distance_mm);
        }
        else{
            SmartDashboard.putNumber("algaeMeasurement", -1);
        }
   
        if(coralLaserMeasurement != null){
            SmartDashboard.putNumber("coralMeasurement", coralLaserMeasurement.distance_mm);
        }
        else{
            SmartDashboard.putNumber("coralMeasurement", -1);
        }

        SmartDashboard.putBoolean("Algae In Range", algaeSensed());
        SmartDashboard.putBoolean("Coral In Range", coralSensed());
        SmartDashboard.putBoolean("Infeed State", infeedState);
        SmartDashboard.putString("Algae Infeed State", getAlgaeInfeedState());

        SmartDashboard.putBoolean("AlgaeReefSensed", algaeReefSensed());
        SmartDashboard.putBoolean("ReefSensed", reefSensed());

        SmartDashboard.putNumber("Algae Ambient", algaeLaserMeasurment.ambient);
        SmartDashboard.putNumber("Algae Status", algaeLaserMeasurment.status);
        SmartDashboard.putBoolean("Valid Algae", validAlgaeMeasurment());

        SmartDashboard.putNumber("Left Ambient", leftReefMeasurement.ambient);
        SmartDashboard.putNumber("Left Status", leftReefMeasurement.status);
        SmartDashboard.putBoolean("Valid Left", validLeftReefMeasurment());

        SmartDashboard.putNumber("Right Ambient", rightReefMeasurement.ambient);
        SmartDashboard.putNumber("Right Status", rightReefMeasurement.status);
        SmartDashboard.putBoolean("Valid Right", validRightReefMeasurment());

        SmartDashboard.putNumber("LeftReef", leftReefMeasurement.distance_mm);
        SmartDashboard.putNumber("RightReef", rightReefMeasurement.distance_mm);

    }


    public boolean algaeSensed(){
        boolean algaeInRange;
        if(algaeLaserMeasurment.distance_mm > 0.00000001 && algaeLaserMeasurment.distance_mm < 100){
            algaeInRange = true;
        }
        else{
            algaeInRange = false;
        }
        return algaeDebouncer.calculate(algaeInRange);
    }

    public boolean algaeReefSensed(){
        boolean algaeInRange;
        if(algaeLaserMeasurment.distance_mm < 200 && validAlgaeMeasurment()){
            algaeInRange = true;
        }
        else{
            algaeInRange = false;
        }
        return algaeInRange;
    }

    public boolean reefSensed(){
        boolean leftRange;
        boolean rightRange;
        boolean inRange;

        if(leftReefMeasurement.distance_mm < 270 && validLeftReefMeasurment()){
            leftRange = true;
        }
        else{
            leftRange = false;
        }

        if(rightReefMeasurement.distance_mm < 270 && validRightReefMeasurment()){
            rightRange = true;
        }
        else{
            rightRange = false;
        }

        if(rightRange && leftRange){
            inRange = true;
        }
        else{
            inRange = false;
        }

        return inRange;
    }


    
    public boolean algaeInfeedDelay(){
        return algaeInfeedDelay.calculate(algaeSensed());
    }

    public boolean coralSensed(){
        boolean coralInRange;
        if(coralLaserMeasurement.distance_mm < 85){
            coralInRange = true;
        }
        else{
            coralInRange = false;
        }
        return coralDebouncer.calculate(coralInRange);
    }


    private boolean validAlgaeMeasurment(){
        boolean validAlgaeMeasurment;
        if(algaeLaserMeasurment.ambient >= 300 && algaeLaserMeasurment.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validAlgaeMeasurment = false;
        }
        else{
            validAlgaeMeasurment = true;
        }

        return validAlgaeMeasurment;
    }
    private boolean validLeftReefMeasurment(){
        boolean validAlgaeMeasurment;
        if(leftReefMeasurement.ambient >= 300 && leftReefMeasurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validAlgaeMeasurment = false;
        }
        else{
            validAlgaeMeasurment = true;
        }

        return validAlgaeMeasurment;
    }
    private boolean validRightReefMeasurment(){
        boolean validAlgaeMeasurment;
        if(rightReefMeasurement.ambient >= 300 && rightReefMeasurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validAlgaeMeasurment = false;
        }
        else{
            validAlgaeMeasurment = true;
        }

        return validAlgaeMeasurment;
    }


    //Coral
    public boolean coralSourceInfeedDelay(){
        return coralSourceInfeedDelay.calculate(coralSensed());
    }












    public void setEndCoralInfeedCommand(boolean endCommand){
        this.endCommand = endCommand;
    }

    public boolean endCoralInfeedCommand(){
        return endCommand;
    }


    //Algae
    public void setEndAlgaeInfeedCommand(boolean endCommand){
        this.endCommand = endCommand;
    }
        public Boolean endAlgaeInfeedCommand(){
        return endCommand;
    }

    public String getAlgaeInfeedState(){
        return algaeInfeedState; 
    }

    public void setAlgaeInfeedState(String algaeInfeedState){
        this.algaeInfeedState = algaeInfeedState;
    }

    public boolean getInfeedState(){
        return infeedState;
    }

    public void setInfeedState(boolean infeedState){
        this.infeedState = infeedState;
    }

    public void toggleInfeedState(){
        if(infeedState){
            infeedState = false;
        }
        else if(!infeedState){
            infeedState = true;
        }
    }

    public void toggleL4State(){
        if(L4State){
            L4State = false;
        }
        else if(!L4State){
            L4State = true;
        }
    }


    public enum AlgaeInfeedState{
        FLOOR,
        L2,
        L3
    }
    
    AlgaeInfeedState algaeInfeedState2 = AlgaeInfeedState.FLOOR;

    public void incrementAlgaeInfeedState(){

        switch(algaeInfeedInt){
            case 1: algaeInfeedState2 = AlgaeInfeedState.FLOOR; break;
            case 2: algaeInfeedState2 = AlgaeInfeedState.L2; break;
            case 3: algaeInfeedState2 = AlgaeInfeedState.L3; break;

            default: algaeInfeedState2 = AlgaeInfeedState.FLOOR; algaeInfeedInt = 1; break;
        }

        algaeInfeedInt ++;

    }

    public void resetAlgaeInfeedState(){
        algaeInfeedInt = 1;
        algaeInfeedState2 = AlgaeInfeedState.FLOOR;
    }

    
}

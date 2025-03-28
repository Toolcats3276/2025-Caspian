package frc.robot.subsystems;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class SensorSS extends SubsystemBase{

    private LaserCan m_topLaserCAN;
    private LaserCan m_bottomLaserCAN;
    
    private LaserCan m_coralLaserCAN;

    LaserCan.Measurement topLaserCANMeasurment;
    LaserCan.Measurement bottomLaserMeasurment;
    
    LaserCan.Measurement coralLaserMeasurement;

    private Debouncer algaeDebouncer;
    private Debouncer algaeInfeedDelay;

    private Debouncer coralDebouncer;
    private Debouncer coralSourceInfeedDelay;


    private boolean endCommand = false;

    private boolean infeedState;
    
    private String algaeInfeedState = InfeedConstants.ALGAE_INFEED_GROUND;
    private int algaeInfeedInt = 1;
    private boolean L4State;
        

    public SensorSS(){
        // try {
        //     m_bottomLaserCAN.setRegionOfInterest(new RegionOfInterest(8, 8, 4, 4));
        // } catch (ConfigurationFailedException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }
        // try {
        //     m_bottomLaserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        // } catch (ConfigurationFailedException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }

        m_coralLaserCAN = new LaserCan(1);
        coralLaserMeasurement = m_coralLaserCAN.getMeasurement();

        coralDebouncer = new Debouncer(0.1);
        coralSourceInfeedDelay = new Debouncer(0.5);


        m_topLaserCAN = new LaserCan(2);
        topLaserCANMeasurment = m_topLaserCAN.getMeasurement();

        m_bottomLaserCAN = new LaserCan(3);
        bottomLaserMeasurment = m_bottomLaserCAN.getMeasurement();

        algaeDebouncer = new Debouncer(0.1);
        algaeInfeedDelay = new Debouncer(3);
        
    }

    @Override
    public void periodic() {
        topLaserCANMeasurment = m_topLaserCAN.getMeasurement();
        bottomLaserMeasurment = m_bottomLaserCAN.getMeasurement();

        coralLaserMeasurement = m_coralLaserCAN.getMeasurement();
   
        if(coralLaserMeasurement != null){
            SmartDashboard.putNumber("coralMeasurement", coralLaserMeasurement.distance_mm);
        }
        else{
            SmartDashboard.putNumber("coralMeasurement", -1);
        }

        SmartDashboard.putBoolean("Top Algae Sensed", topAlgaeSensed());
        SmartDashboard.putBoolean("Bottom Algae Sensed", bottomAlgaeSensed());

        SmartDashboard.putBoolean("Coral Sensed", coralSensed());

        SmartDashboard.putBoolean("Reef Sensed", reefSensed());

        SmartDashboard.putNumber("Top Measurment", topLaserCANMeasurment.distance_mm);
        SmartDashboard.putNumber("Top Ambient", topLaserCANMeasurment.ambient);
        SmartDashboard.putNumber("Top Status", topLaserCANMeasurment.status);
        SmartDashboard.putBoolean("Valid Top", validTopMeasurment());

        SmartDashboard.putNumber("Bottom Measurment", bottomLaserMeasurment.distance_mm);
        SmartDashboard.putNumber("Bottom Ambient", bottomLaserMeasurment.ambient);
        SmartDashboard.putNumber("Bottom Status", bottomLaserMeasurment.status);
        SmartDashboard.putBoolean("Valid Bottom", validTopMeasurment());
        


    }

    public boolean topAlgaeSensed(){
        boolean algaeInRange;
        if(topLaserCANMeasurment.distance_mm < 100 && validBottomMeasurment()){
            algaeInRange = true;
        }
        else{
            algaeInRange = false;
        }
        return algaeInRange;
    }

    public boolean bottomAlgaeSensed(){
        boolean algaeInRange;
        if(bottomLaserMeasurment.distance_mm < 100 && validBottomMeasurment()){
            algaeInRange = true;
        }
        else{
            algaeInRange = false;
        }
        return algaeInRange;
    }


    public boolean reefSensed(){
        boolean topRange;
        boolean bottomRange;
        boolean inRange;

        if(topLaserCANMeasurment.distance_mm < 500 && validTopMeasurment()){
            topRange = true;
        }
        else{
            topRange = false;
        }

        if(bottomLaserMeasurment.distance_mm < 500 && validBottomMeasurment()){
            bottomRange = true;
        }
        else{
            bottomRange = false;
        }

        if(bottomRange && topRange){
            inRange = true;
        }
        else{
            inRange = false;
        }

        return inRange;
    }

    
    public boolean algaeInfeedDelay(){
        return algaeInfeedDelay.calculate(bottomAlgaeSensed());
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


    private boolean validTopMeasurment(){
        boolean validMeasurment;
        if(topLaserCANMeasurment.ambient >= 270 && topLaserCANMeasurment.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validMeasurment = false;
        }
        else{
            validMeasurment = true;
        }

        return validMeasurment;
    }

    private boolean validBottomMeasurment(){
        boolean validMeasurment;
        if(bottomLaserMeasurment.ambient >= 270 && bottomLaserMeasurment.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validMeasurment = false;
        }
        else{
            validMeasurment = true;
        }

        return validMeasurment;
    }


    private boolean validMeasurment(Measurement laserCANMeasurment){
        boolean validMeasurment;
        if(laserCANMeasurment.ambient >= 300 && laserCANMeasurment.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validMeasurment = false;
        }
        else{
            validMeasurment = true;
        }

        return validMeasurment;
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

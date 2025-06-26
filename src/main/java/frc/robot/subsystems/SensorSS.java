package frc.robot.subsystems;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class SensorSS extends SubsystemBase{

    private LaserCan m_topLaserCAN;
    private LaserCan m_bottomLaserCAN;
    
    private LaserCan m_coralLaserCAN;

    LaserCan.Measurement topLaserCANMeasurment;
    LaserCan.Measurement bottomLaserCANMeasurment;
    
    LaserCan.Measurement coralLaserMeasurement;

    private Debouncer algaeInfeedDelay;

    private Debouncer coralDebouncer;
    private Debouncer coralSourceInfeedDelay;


    private boolean endCommand = false;

    private boolean infeedState;
    
    private String algaeInfeedState = InfeedConstants.ALGAE_INFEED_GROUND;
    private int algaeInfeedInt = 1;
    private boolean L4State;
        

    public SensorSS(){
        m_coralLaserCAN = new LaserCan(1);
        coralLaserMeasurement = m_coralLaserCAN.getMeasurement();

        coralDebouncer = new Debouncer(0.1);
        coralSourceInfeedDelay = new Debouncer(0.5);


        m_topLaserCAN = new LaserCan(2);
        topLaserCANMeasurment = m_topLaserCAN.getMeasurement();

        m_bottomLaserCAN = new LaserCan(3);
        bottomLaserCANMeasurment = m_bottomLaserCAN.getMeasurement();

        algaeInfeedDelay = new Debouncer(3);
        
    }

    @Override
    public void periodic() {
        topLaserCANMeasurment = m_topLaserCAN.getMeasurement();
        bottomLaserCANMeasurment = m_bottomLaserCAN.getMeasurement();

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
        SmartDashboard.putBoolean("Top Reef", topLaserCANMeasurment.distance_mm < 600 && topLaserCANMeasurment.distance_mm > 0.01 && validTopMeasurment());
        SmartDashboard.putBoolean("Valid Top", validTopMeasurment());

        SmartDashboard.putNumber("Bottom Measurment", bottomLaserCANMeasurment.distance_mm);
        SmartDashboard.putNumber("Bottom Ambient", bottomLaserCANMeasurment.ambient);
        SmartDashboard.putNumber("Bottom Status", bottomLaserCANMeasurment.status);
        SmartDashboard.putBoolean("Bottom Reef", bottomLaserCANMeasurment.distance_mm < 600 && bottomLaserCANMeasurment.distance_mm > 0.01 && validBottomMeasurment());
        SmartDashboard.putBoolean("Valid Bottom", validBottomMeasurment());
        
        SmartDashboard.putBoolean("Enabled", DriverStation.isEnabled());
        SmartDashboard.putBoolean("InfeeState", infeedState);

    }

    public boolean topAlgaeSensed(){
        boolean algaeInRange;
        if(topLaserCANMeasurment.distance_mm < 100 && validTopMeasurment()){
            algaeInRange = true;
        }
        else{
            algaeInRange = false;
        }
        return algaeInRange;
    }

    public boolean bottomAlgaeSensed(){
        boolean algaeInRange;
        if(bottomLaserCANMeasurment.distance_mm < 100 && validBottomMeasurment()){
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

        if(topLaserCANMeasurment.distance_mm < 600 && topLaserCANMeasurment.distance_mm > 0.01 && validTopMeasurment()){
            topRange = true;
        }
        else{
            topRange = false;
        }

        if(bottomLaserCANMeasurment.distance_mm < 600 && bottomLaserCANMeasurment.distance_mm > 0.01 && validBottomMeasurment()){
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
        if(topLaserCANMeasurment.ambient <= 400 && topLaserCANMeasurment.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validMeasurment = true;
        }
        else{
            validMeasurment = false;
        }

        return validMeasurment;
    }

    private boolean validBottomMeasurment(){
        boolean validMeasurment;
        if(bottomLaserCANMeasurment.ambient <= 400 && bottomLaserCANMeasurment.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            validMeasurment = true;
        }

        else{
            validMeasurment = false;
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

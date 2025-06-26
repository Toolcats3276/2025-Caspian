
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSS extends SubsystemBase{

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private Color LED_COLOR;  


  private LEDPattern gold = LEDPattern.solid(Color.kGold);
  private LEDPattern green = LEDPattern.solid(Color.kGreen);
  private LEDPattern blanchedAlmond = LEDPattern.solid(Color.kBlanchedAlmond);
  private LEDPattern blink = gold.blink(Seconds.of(1)).overlayOn(blanchedAlmond);
  private LEDPattern aligned = green.blink(Seconds.of(.15)).overlayOn(blanchedAlmond);
  private LEDPattern aligning = green.breathe(Seconds.of(.25));

  
    public LEDSS() {
      m_ledBuffer = new AddressableLEDBuffer(43);
      m_led = new AddressableLED(9);
      m_led.setLength(m_ledBuffer.getLength());
      m_led.start();

      
    }
    
    public enum Mode{
      off, 
      on,
      Infeeding,
      Infeed_Done,
      Coral, 
      Algae,
      ReefAligned,
      climbed,
      climbing,
      idle, 
      autoAligning,
      aligned
    }
    
    Mode LED_Mode = Mode.off;
    
    @Override
    public void periodic() {
      
      var alliance = DriverStation.getAlliance();

        
      switch(LED_Mode){
  
        case off:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++) {
            //sets rgb value
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
          break;
        }
  
        case on:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, LED_COLOR);
          }
          break;
        }
  
        case Infeeding:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kBlanchedAlmond);
          }
          break;
        }
  
        case Infeed_Done:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            // infeedBlink.applyTo(m_ledBuffer);
            m_ledBuffer.setLED(i, Color.kGreen);  
          }
          break;
        }
        
        case Coral:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kBlanchedAlmond);
          }
          break;
        }

        case Algae:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kDarkGreen);
          }
          break;
        }

        case ReefAligned:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kMagenta);
          }
          break;
        }
  
        case climbed:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kBlack);
          }
          break;
        }
  
        case climbing:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            blink.applyTo(m_ledBuffer);
          }
          break;
        }

        case autoAligning:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
              aligning.applyTo(m_ledBuffer);
          }
          break;
        }
        
        case aligned:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            aligned.applyTo(m_ledBuffer);
          }
          break;
        }

        case idle:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
              if (alliance.get() == DriverStation.Alliance.Blue) {
                m_ledBuffer.setLED(i, Color.kBlue);
              }
              else if (alliance.get() == DriverStation.Alliance.Red) {
                m_ledBuffer.setLED(i, Color.kRed);
              } 
              else if (alliance.get() == null){
                m_ledBuffer.setLED(i, Color.kWhite);
              }  
          }
        }

  
      }
  
      m_led.setData(m_ledBuffer);
  
    }
  
    public void off(){
      LED_Mode = Mode.off;
      System.out.println("Off");
      m_led.setData(m_ledBuffer);
    }
  
    public void on(Color LED_COLOR){
      LED_Mode = Mode.on;
      this.LED_COLOR = LED_COLOR;
    m_led.setData(m_ledBuffer);
  }

  public void Infeeding(){
    LED_Mode = Mode.Infeeding;
    m_led.setData(m_ledBuffer);
  }

  public void Infeed_Done(){
    LED_Mode = Mode.Infeed_Done;
    m_led.setData(m_ledBuffer);
  }

  // public void rainbow() {
  //   LED_Mode = Mode.rainbow;
  //   System.out.println("Rainbow");
  //   m_led.setData(m_ledBuffer);
  // }

  public void CoralSensed(){
    LED_Mode = Mode.Coral;
    m_led.setData(m_ledBuffer);
  }

  public void AlgaeSensed(){
    LED_Mode = Mode.Algae;
    m_led.setData(m_ledBuffer);
  }
  
  public void ReefAligned(){
    LED_Mode = Mode.ReefAligned;
    m_led.setData(m_ledBuffer);
  }

  public void climbed(){
    LED_Mode = Mode.climbed;
    m_led.setData(m_ledBuffer);
  }

  public void climbing(){
    LED_Mode = Mode.climbing;
    m_led.setData(m_ledBuffer);
  }

  public void idle(){
    LED_Mode = Mode.idle;
    m_led.setData(m_ledBuffer);
  }

  public void autoAligning(){
    LED_Mode = Mode.autoAligning;
    m_led.setData(m_ledBuffer);
  }

  public void aligned(){
    LED_Mode = Mode.aligned;
    m_led.setData(m_ledBuffer);
  }

}
package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(18);
    private LEDPattern red = LEDPattern.solid(Color.kGreen);
    private LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private LEDPattern green = LEDPattern.solid(Color.kRed);
    private LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    private LEDPattern pink = LEDPattern.solid(Color.kTeal);
    private LEDPattern blinkred = LEDPattern.solid(Color.kGreen).blink( Seconds.of(0.1));
    private LEDPattern blinkblue = LEDPattern.solid(Color.kBlue).blink( Seconds.of(0.1));
    private LEDPattern blinkgreen = LEDPattern.solid(Color.kRed).blink( Seconds.of(0.1));
    private LEDPattern blinkyellow = LEDPattern.solid(Color.kYellow).blink( Seconds.of(0.1));
    private LEDPattern blinkpink = LEDPattern.solid(Color.kTeal).blink( Seconds.of(0.1));
    
    public LED (){
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
    }

    public void SolidGreen(){
        green.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidRed(){
        red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidBlue(){
        blue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidYellow(){
        yellow.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidPink(){
        pink.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkGreen(){
        blinkgreen.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkRed(){
        blinkred.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkBlue(){
        blinkblue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkYellow(){
        blinkyellow.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkPink(){
        blinkpink.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    

}
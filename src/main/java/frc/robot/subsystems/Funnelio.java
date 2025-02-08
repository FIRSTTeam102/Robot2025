package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
// TODO Add imports for... sensor

public class Funnelio {
    private DigitalInput funnelSensor = new DigitalInput(Constants.FunnelConstants.FUNNELSENSOR);


    public boolean getfunnelSensor(){
    return funnelSensor.get();
    }
    // TODO Recognize if sensor says when coral isn’t in the funnel

    // TODO Recognize if the sensor says coral IS in the funnel

}

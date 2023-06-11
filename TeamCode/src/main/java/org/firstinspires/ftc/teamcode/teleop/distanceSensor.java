package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class distanceSensor {
        private AnalogInput analog;
        private int maxRange = 520;
        public distanceSensor(AnalogInput analog) {
            this.analog = analog;
        }

        public double getDistance() {
            double distance = (this.analog.getVoltage() * maxRange) / 3.3;
            return distance;
        }
}

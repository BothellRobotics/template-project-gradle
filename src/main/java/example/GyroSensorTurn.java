package example;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.sensors.ev3.EV3GyroSensor;
import ev3dev.sensors.ev3.EV3UltrasonicSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroSensorTurn {

	public static void main(String[] args) {
		System.out.println("Creating Motor A & D");
        final EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        final EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
        Logger LOGGER = LoggerFactory.getLogger(ShuttleBetweenObstruction.class);
        final EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
        final EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
        
        //To Stop the motor in case of pkill java for example
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
            public void run() {
                System.out.println("Emergency Stop");
                motorLeft.stop();
                motorRight.stop();
            }
        }));
        
        //Ultrasonic Sample provider        
        final SampleProvider sp = us1.getDistanceMode();
		int distanceValue = 0;
		float [] sample = new float[sp.sampleSize()];
		
		for(int j = 0; j <10; j++) {
			sample = new float[sp.sampleSize()];
			sp.fetchSample(sample, 0);
	        distanceValue = (int) sample[0];
	        
	        LOGGER.info("Distance: {}",  distanceValue);
	        
	        Delay.msDelay(500);
		}
		
		gyroSensor.reset();
		final SampleProvider gyroSensorSample = gyroSensor.getAngleMode();
		int angleValue = 0;
		float [] angleSample = new float[gyroSensorSample.sampleSize()];
		
		for(int j = 0; j <10; j++) {
			angleSample = new float[gyroSensorSample.sampleSize()];
			gyroSensorSample.fetchSample(angleSample, 0);
			angleValue = (int) angleSample[0];
	        
	        LOGGER.info("Angle: {}",  angleValue);
	        
	        Delay.msDelay(500);
		}
		
		boolean moveForward = true;
		for(int i = 0; i < 10; i++) {
			LOGGER.info("Interation: {}", i);
			while(moveForward) {				
				sample = new float[sp.sampleSize()];
	            sp.fetchSample(sample, 0);
	            distanceValue = (int) sample[0];
	            
	            if(distanceValue < 50) {
	            	 System.out.println("Stop motors");
	            	 LOGGER.info("Before Stop - Distance from obstacle: {}", distanceValue);
	                 motorLeft.stop();
	                 motorRight.stop();  
	                 moveForward = false;
	            }
	            else {
	            	LOGGER.info("Going Forward - Distance from obstacle: {}", distanceValue);
	            	System.out.println("Go Forward with the motors");
	            	motorLeft.setSpeed(500);
	                motorRight.setSpeed(500);
	                
			        motorLeft.forward();
			        motorRight.forward();
			        
			        Delay.msDelay(250);
			        moveForward = true;
	            }
			}
			
			//U-Turn
			motorLeft.setSpeed(100);
	        motorRight.setSpeed(100);
	        gyroSensor.reset();
	        Delay.msDelay(100);
	        angleSample = new float[gyroSensorSample.sampleSize()];
			gyroSensorSample.fetchSample(angleSample, 0);
			angleValue = (int) angleSample[0];
			LOGGER.info("Angle before turning: {}", angleValue);
			
	        while(true) {
	        	LOGGER.info("Make U-Turn");
		        motorLeft.forward();
		        motorRight.backward();
		        Delay.msDelay(50);
		        
		        angleSample = new float[gyroSensorSample.sampleSize()];
				gyroSensorSample.fetchSample(angleSample, 0);
				int recentAngleValue = (int) angleSample[0];
				
				LOGGER.info("Angle after turning for 50 ms: {}", recentAngleValue);
				
				if(Math.abs(angleValue - recentAngleValue) < 180) {
					continue;
				}
				else {
					motorLeft.stop();
					motorRight.stop();
					break;
				}					
	        }
	        
        	moveForward = true;
		}
	}

}

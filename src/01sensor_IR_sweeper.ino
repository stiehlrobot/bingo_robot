// write the code for the IR sweeper algorithm that controls servo sweep and IR interrupter
// publish the binary data from the IR interrupter to Sensor_msgs/IR




void setup {
    
    sensor_height_from_ground = 15; //define sensor height from ground level in cm
    hypotheneuse_width = 25; //define the hypothenuse distance in cm

    posAmount = 20; //how many beams is the laser divided into 
    chasm_position = Null;

    servoMaxRange = 90;
    servoMinRange = 0;
    posIncrement = servoMaxRange / (posAmount - 1);
    delay_between_positions = 100; //delay between servo positions in milliseconds

    int pos = 0;

}

void loop {

    //v1 loop test
    for (pos; pos <= servoMaxRange; pos + posIncrement) {
        if (digitalRead(IR_PIN) == LOW) {

            chasm_position = pos; 
            //publish this chasm position via ros publisher

        }

    for (pos; pos ǵreatertan= servoMinRange; pos - posIncrement) {
        if (digitalRead(IR_PIN) == LOW) {

            chasm_position = pos; 
            //publish this chasm position via ros publisher

        }

}
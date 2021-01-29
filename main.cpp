#include <phidget22.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <unistd.h>


/*
@function: get_error()
@param PhidgetVoltageInputHandle: position channel
@param int: reference
@return double: error
Calculates the error based on reference value and measured position
*/
double get_error(PhidgetVoltageInputHandle position_channel, double reference){
    
    double current_pos;
    double error;

    // Get current measured position
    PhidgetVoltageInput_getVoltage(position_channel, &current_pos); 
    // Calculate the error
    error = -((double)reference - (double)current_pos);  

    printf("Error: %lf\n", error);

    return error;
}


/*
@function: regulator()
@param double: Kp --- Gain
@param double: Ti --- Integrator parameter
@param double: Td --- Derivative parameter
@param double: error --- deviance from reference position
@param double: dt --- timestep
@param double: *integral_pointer --- Value of the integral
@param double: *pre_error_pointer --- previous error value
@return double: controller --- controller output
The function calculates the new thrust, but if the controller output exceeds the limits of the motor, 
the thrust is set to the limits.
*/
double regulator(double Kp, double Ki, double Kd, double error, double dt,  double *integral_pointer, double *pre_error_pointer){
    double derivative;
    double thrust;
    double integral = *integral_pointer;
    double pre_error = *pre_error_pointer;
    // Calculate the integral term
    integral = integral + error*dt;
    printf("integral: %lf \n dt = %f\n", integral, dt); 
    // Calculate the derivative term
    derivative = (error - pre_error)/dt;
    printf("Derivative: %lf \n", derivative);
    // Calculate new thrust as controller output
    thrust = Kp*error + Ki*integral + Kd*derivative;
    printf("Controller output = %f \n", thrust);
    // Store the error for caluculation of the derivative in the next iteration
    pre_error = error;
    *integral_pointer = integral;
    
    return thrust;
}

/*
@function: set_thrust()
@param PhidgetRCServoHandle: rcServo --- Servo channel handle
@param double: thrust --- Calculated new thrust value from the controller
@param double: dt --- timestep
The function sets the new thrust and engages, and lets the thrust be activated 
for the chosen time.
*/
void set_thrust(PhidgetRCServoHandle rcServo, double thrust, double dt){

    // The timestep has to be in micro seconds
    int sleep = 1000000*dt; 

    // Limits the thrust(based on the note in the lab)
    if(thrust > 169){
        thrust = 169;
        printf("Controller output exceeded the limits and is set to %f \n", thrust);
    }
    else if(thrust < 110){
        thrust = 110;
        printf("Controller output exceeded the limits and is set to %f \n", thrust);
    }
    
    PhidgetRCServo_setTargetPosition(rcServo, thrust);
    PhidgetRCServo_setEngaged(rcServo, 1);

    usleep(sleep); 
}


/*
@function: manage_time()
@param double: *elapsed_time_pointer --- The pointer to the elapsed time variable
@param double: *total_time_pointer --- The pointer to the total time variable
@param double: start_time --- the time of when the program starts running
@param double: sleeptime --- seconds slept while activating the phidget devices
The function updates the total time, for use in analysis
*/
void manage_time(double *elapsed_time_pointer, double *total_time_pointer, double start_time, double sleeptime){

    *elapsed_time_pointer = ((double) (clock()- start_time)/CLOCKS_PER_SEC) + sleeptime; 
    *total_time_pointer = *total_time_pointer + *elapsed_time_pointer;

    printf("Elapsed time: %f \n", *elapsed_time_pointer);
    printf("Total time: %f \n", *total_time_pointer);
}


/*
@function: Write_reult()
@param double: total_time --- Total run time
@param double: error --- deviance from reference position
@param PhidgetVoltageInputHandle: position --- voltage input handle for measuring of position
Writes time, position and error to the  results.dat-file
*/
void write_result(double total_time, double error, PhidgetVoltageInputHandle position){
    double current_position;
    PhidgetVoltageInput_getVoltage(position, &current_position);
    FILE *f = fopen("results.dat", "a");
    fprintf(f, "%lf %lf %lf \n", total_time, error, current_position);
    fclose(f);
}


/*
@function
writes the results.dat-file to .vtf-file to be used by GLView
*/
void write_vtf(){
    
    //Open files
    FILE *f = fopen("dynpos.vtf", "a");
    FILE *fr = fopen("results.dat", "r");
    int i=1, j, c;
    double t, error, new_pos;



    //Set up vessel
    fprintf(f,"*VTF-1.00\n\n\n");
    fprintf(f,"!vessel:\n");
    fprintf(f,"*NODES 1\n");
    fprintf(f,"0. 12. 0.\n");
    fprintf(f,"150. 12. 0.\n");
    fprintf(f,"150. -12. 0.\n");
    fprintf(f,"0. -12. 0.\n");
    fprintf(f,"0. 12. 12.\n");
    fprintf(f,"150. 12. 12.\n");
    fprintf(f,"150. -12. 12.\n");
    fprintf(f,"0. -12. 12.\n\n\n");

    fprintf(f,"!pool:\n");
    fprintf(f,"*NODES 2\n");
    fprintf(f,"0. 50. 0.\n");
    fprintf(f,"350. 50. 0.\n");
    fprintf(f,"350. -50. 0.\n");
    fprintf(f,"0. -50. 0.\n\n\n");

    fprintf(f,"*ELEMENTS 1\n");
    fprintf(f,"%%NODES #1\n");
    fprintf(f,"%%HEXAHEDRONS\n");
    fprintf(f,"1 2 3 4 5 6 7 8\n\n\n");

    fprintf(f,"*ELEMENTS 2\n");
    fprintf(f,"%%NODES #2\n");
    fprintf(f,"%%QUADS\n");
    fprintf(f,"1 2 3 4\n\n\n");

    fprintf(f,"*GLVIEWGEOMETRY 1\n");
    fprintf(f,"%%ELEMENTS\n");
    fprintf(f,"1, 2\n\n\n");

    while((c = fgetc(fr)) != EOF) {
        //Read new pos
        fscanf(fr,"%lf %lf %lf", &t, &error, &new_pos);

        //Write to vtf file
        fprintf(f,"*RESULTS %lf\n",i);

        fprintf(f,"%%DIMENSION 3\n");
        fprintf(f,"%%PER NODE #1\n");
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n",new_pos);
        fprintf(f,"%lf 0 0\n\n\n",new_pos);

        ++i;
        }

    fprintf(f,"*GLVIEWVECTOR 1\n");
    fprintf(f,"%%NAME DISPLACEMENT\n");

    for(j=1;j<i;++j){
        fprintf(f,"%%STEP %d\n",j);
        fprintf(f,"%d\n",j);
        }


    fclose(f);
    fclose(fr);
    }

/*
@function: setReference()
@param PhidgetVoltageInputHandle: pos --- The voltage input handle for position
@param double: percentage --- The percentage of the differanse between the origin and the maximum position, we want
                              the boat to travel.
@return double: reference --- The reference value calculated
The function setReference calculates the reference position based on maximum value for position, the origin position
and a percentage of the differanse between them. 
*/
double setReference(PhidgetVoltageInputHandle pos, double percentage){
    // Set origin position
    double origin_pos;
    printf("\nWhen in origin position: ");
    system("pause");
    PhidgetVoltageInput_getVoltage(pos, &origin_pos);
    printf("%lf", origin_pos);
    // Set maximum position
    double max_pos;
    printf("\nWhen in maximum position: ");
    system("pause");
    PhidgetVoltageInput_getVoltage(pos, &max_pos);  
    printf("%lf", max_pos); 
    // Calculates reference position
    double reference = origin_pos + percentage*(max_pos - origin_pos);
    return reference;
}

/*
@function: main()
The main function sets intitial value, the channels needed for I/O, runs the program and writes the results
to the vtf-file.
*/
int main(){
    printf("Wait for set up.............\n");
    // Create servo channel
    PhidgetRCServoHandle rcServo;
    PhidgetRCServo_create(&rcServo);

    // Create position channel
    PhidgetVoltageInputHandle pos;
    PhidgetVoltageInput_create(&pos);

    
    Phidget_setDeviceSerialNumber((PhidgetHandle) pos, 85753); // Set the device serialnumber
    Phidget_setChannel((PhidgetHandle)pos, 1); // Choose which channel to use
    Phidget_openWaitForAttachment((PhidgetHandle)pos, 5000); // Open and wait for attachment(usb)
   

    Phidget_setDeviceSerialNumber((PhidgetHandle)rcServo, 42685); // Set the device serialnumber
    Phidget_openWaitForAttachment((PhidgetHandle)rcServo, 5000); // Open and wait for attachment(usb)
    

    // remove previous result files
    remove("results.dat");
    remove("dynpos.vtf");

    
    // set intial values:
    double dt = 0.2; 
    double total_time = 0;
    double elapsed_time;
    double integral = 0;
    double pre_error = 0; 

    printf("set up is done!\n");

    // Get reference from user
    double percentage;
    double reference;
    printf("\nSet percentage of distance as referance(number between 0-1): \n");
    scanf("%lf", &percentage);
    printf("\nYou entered: %lf\n", percentage);
    reference = setReference(pos, percentage);
    printf("\nThe reference is %lf \n", reference);

    // Get gain(Kp) from user
    double Kp;
    printf( "\nSet gain(Kp): \n");
    scanf("%lf", &Kp);
    printf( "\nYou entered: %lf \n", Kp);

    // Get integral gain(Ki) from user
    double Ki;
    printf( "\nSet integral parameter(Ki): \n");
    scanf("%lf", &Ki);
    printf( "\nYou entered: %lf\n", Ki);

    // Get derivative gain(Td) from user
    double Kd;
    printf( "\nSet derivate parameter(Kd): \n");
    scanf("%lf", &Kd);
    printf( "\nYou entered: %lf \n", Kd);

    // Get the maximum run time from user
    double max_run_time;
    printf( "\nset maximum run time: \n");
    scanf("%lf", &max_run_time);
    printf( "\nYou entered: %lf \n", max_run_time);

    // Run the program
    while(total_time <= max_run_time){
        // Set start time
        int start_time = clock(); 
    
        // Calculate error
        double error = get_error(pos, reference); 
        
        // Calculate new thrust value
        double new_thrust = regulator(Kp, Ki, Kd, error, dt, &integral, &pre_error); 
        printf("The new thrust is: %lf \n", new_thrust);

        // set new thrust value
        set_thrust(rcServo, new_thrust, dt);

        // manage time
        manage_time(&elapsed_time, &total_time, start_time, dt);

        // write result to file
        write_result(total_time, error, pos);
    }

    // Close and delete all channels
    Phidget_close((PhidgetHandle) rcServo);
    PhidgetRCServo_delete(&rcServo);
    Phidget_close((PhidgetHandle) pos);
    PhidgetVoltageInput_delete(&pos);

    // write .vtf file
    write_vtf();
    return 0;
}
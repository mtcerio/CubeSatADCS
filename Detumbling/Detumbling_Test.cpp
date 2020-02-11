/**
 * @file Detumbling_Test.cpp
 * @version 1.0
 * @date 2020
 * @author Remy CHATEL & Patrik Pauliny
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for Detumbling.test.h
 * 
 * @see Detumbling.test.h
 * 
 * # License
 * <b>(C) Copyright 2019 Patrik PAULINY</b>
 * 
 * Licensed Under  GPL v3.0 License
 * http://www.gnu.org/licenses/gpl-3.0.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "Detumbling.test.h"
#include "mbed.h"
#include "Matrix.h"
#include "MPU9150.h"
#include "Filters.h"

int DetumblingTest(){
    I2C i2c(D4, D5);
    i2c.frequency(400000);
    MPU9150 imu(&i2c);
    Timer t;
    Timer SensorInterval;
    t.start();
    Timer Interval;

    //--------------------- INIT ---------------------//

    //init Bluetooth conection for data logging 
    Serial bluetooth(D1,D0, 9600);
    printf("\r\n\r\n\r\n\r\n\r\n\r\n");
    printf("----------------------------------------\r\n");
    printf("Bluetooth Connection ok\r\n");

    int ellapsed = 0;
    int last_update = 0;
    int print_update = t.read_ms();

    float rmag_b[3];
    float rmag_e[3] = {17.5, 0.5, 47};

    int mcount = 0; // Frequency divider for the magnetometer
    uint8_t MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values (doesn't really work! 100=60)
    uint8_t acc_scale = AFS_2G;
    uint8_t gyr_scale = GFS_250DPS;
    float val_acc[3], val_gyr[3], val_mag[3], ControlOut[3], B_dot_data[3], MagneticField_data[3]; 
    
    //Init DACs for Magnetorquer Driver voltage reference 
    PwmOut Aout1(A5); // Using an external DAC to get analog reference voltage (not enought AnalogOut pins)
    AnalogOut Aout2(A4);
    AnalogOut Aout3(A3);

    //Init Digitalout for control of direction of the torque (H-Bridge)
    //                       forward     reverse      Break 
    DigitalOut Cont11(D2); //   1           0           1  
    DigitalOut Cont12(D3); //   0           1           1  
    DigitalOut Cont21(D6); //   1           0           1  
    DigitalOut Cont22(D9); //   0           1           1  
    DigitalOut Cont31(D10); //  1           0           1  
    DigitalOut Cont32(D11); //  0           1           1   

    //Maximum Dipole moment available from Magnetorquers 
    float Max_Voltage = 3.3; 
    
    //Variables for calculating avrage of magnetic field 
    Matrix Avrage = Matrix::zeros(3,1);
    Matrix SumAll = Matrix::zeros(3,1);

    // variables for calculating B-dot 
    Matrix val_mag_now = Matrix::zeros(3,1);
    Matrix val_mag_previous = Matrix::zeros(3,1);
    Matrix B_dot = Matrix::zeros(3,1);
    Matrix Cont_Out = Matrix::zeros(3,1);
    float Control_gain[3] = {0.4, 0.4, 0.4};
    Matrix Cont_gain(1,3, Control_gain);

    // IIR filter variables 
    Matrix AccPrevius = Matrix::zeros(3,1);
    Matrix Delay1_previous = Matrix::zeros(3,1);
    Matrix Delay2_previous = Matrix::zeros(3,1);
    Matrix AccOUT_NOW = Matrix::zeros(3,1);

    //IIR Low pass Filter Coeficients 
    float a1= -1.56101808;                  
    float a2=  0.64135154;                    
    float b0= 0.02008337; //3Hz low pass    
    float b1= 0.04016673;                   
    float b2= 0.02008337;                   

    
    //Init Low-pass Buterworth IIR filter 
    Filters::IIRFilter IIR(a1, a2, b0, b1, b2);  

    int Iterator= 0;
    float SensInter= 0; 
    
    // IMU connection and initial calibration 
    printf("\n\r\n\r\n\r\n\r\n\r\n\r--------------------------------------\n\r");
    printf("IMU Connection ok\n\r");
    uint8_t whoami = imu.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68 or 0x73\n\r");
    if (!imu.initIMU(acc_scale, gyr_scale)) {   // WHO_AM_I should be 0x68
        printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }
    //Calibration 
    imu.recalibrateIMU(10000, 1000);
    float null_avg[3] = {0,0,0};
    imu.setAvgAcc(null_avg);
    imu.setAvgMag(null_avg);
    printf("IMU ok\n\r");

    //Matrix dq(4,1);
    //Matrix gyr(3,1);
    //Matrix quat = Matrix::zeros(4,1);
    //quat(2) = 1;
    //float dt2, w_norm;
    //int time = t.read_us();
   
    //--------------------- LOOP ---------------------//
    Interval.start();
    while(1){    
        SensInter=0; //reset Timer 
        Iterator= 0; //reset iterator 
        SensorInterval.start();
        imu.setAvgMag(null_avg);
        SumAll = Matrix::zeros(3,1);
        Avrage = Matrix::zeros(3,1);

        // turn magnetorquers off to avoid compromising the magnetic field measurement 
        Aout1.write(0.01);
        Aout2.write(0);
        Aout3.write(0);
        wait_us(10);// wait till the magnetic field generated by the magnetorquers decays to zero 

        // measurement Period begins 
        while (SensInter <500){
            imu.setAvgMag(null_avg);
            if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
                imu.getAccel(val_acc);  // Read the x/y/z adc values
                imu.getGyro(val_gyr);  // Read the x/y/z adc values degrees/s 
                mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
                imu.getMag(val_mag);  // Read the x/y/z adc values in uT 
                Matrix Mag_val(3,1,val_mag); // form a matrix so it can be used in the IIR filter 

                //----------------------IIR Butterworth LowPass filter-------------//
                AccOUT_NOW = IIR.Filter(Mag_val, AccPrevius, Delay1_previous, Delay2_previous); // IIR filtering 
                Iterator++; // Store number of samples 
                AccPrevius = IIR.getAccumulatorIN(); 
                Delay1_previous = IIR.getDelay1(); 
                Delay2_previous = IIR.getDelay2();
                SumAll =  AccOUT_NOW + SumAll; // sum all the outputs together to provide avrage value 
                mcount = 0; 
                }
                SensInter= SensorInterval.read_ms(); 
            }
        }
        SensorInterval.stop(); //for determining the measurement frequency accurately 
        SensorInterval.reset();
        //printf("{%u,}\n\r" ,Iterator); 
        Avrage = SumAll/Iterator;
        val_mag_now = Avrage; 
        //--------------------------Calculate B-dot --------------------------------
        B_dot = (val_mag_now - val_mag_previous)/1.5;
        //B_dot.print();
        val_mag_previous = val_mag_now;
        //----------------Calculate Control Output for Magnetorquers----------------
        ControlOut[0]= -Cont_gain.getNumber(0,0) * B_dot.getNumber(0,0); 
        ControlOut[1]= -Cont_gain.getNumber(0,1) * B_dot.getNumber(1,0); 
        ControlOut[2]= -Cont_gain.getNumber(0,2) * B_dot.getNumber(2,0); 
        //printf("{% f,}\n\r",SensInter);

        B_dot.getCoef(B_dot_data);         
        Avrage.getCoef(MagneticField_data);

        //------------ Datat transfer for analysys-------------------------
        float period= Interval.read_ms(); 
        Interval.stop();
        Interval.reset();
        Interval.start();
        bluetooth.printf("Interval, %f, Mag field, %f, %f, %f, B-dot, %f, %f, %f,  ControlOut,%f,%f,%f,Omega,%f,%f,%f, \r\n", period, MagneticField_data[0], MagneticField_data[1], MagneticField_data[2], B_dot_data[0], B_dot_data[1], B_dot_data[2], ControlOut[0], ControlOut[1], ControlOut[2], val_gyr[0], val_gyr[1], val_gyr[2]);
        
        //------------ Debuging prints---------------------------
        //printf("Control Output");
        //printf("{% f, % f, % f}\n\r", ControlOut[0], ControlOut[1], ControlOut[2]);
        //printf("MAG DATA ");
        //printf("%f\t%f\t%f\n", val_mag[0], val_mag[1], val_mag[2]);
        //printf("Acc DATA ");
        //printf("{% 4.2f, % 4.2f, % 4.2f}\n\r", val_acc[0]*1000, val_acc[1]*1000, val_acc[2]*1000);
        //mcount = 0;
                
        //------------Determine direction of torques------------------- 
        if(ControlOut[0]<0){
            ControlOut[0]=-ControlOut[0];
            Cont11= 1;   
            Cont12= 0;
        } else {
            Cont11= 0;   
            Cont12= 1;
        }
        if(ControlOut[1]<0){
            ControlOut[1]=-ControlOut[1];
            Cont21= 1;   
            Cont22= 0;
        }else {
            Cont21= 0;   
            Cont22= 1;
        }
        if(ControlOut[2]<0){
            ControlOut[2]=-ControlOut[2];
            Cont31= 1;   
            Cont32= 0;
        }else{
            Cont31= 0;   
            Cont32= 1;
        }
        //printf("{% f, % f, % f}\n\r", ControlOut[0], ControlOut[1], ControlOut[2]); //
        //------------ Determine necesary Analog/PWM Outputs---------- // 
        
        //---------------

        float Duty= (ControlOut[2]);
        Aout1.period(0.0001f);
        Aout1.write(Duty);
        Aout2.write(ControlOut[0]);
        Aout3.write(ControlOut[1]);
        
        //------------------- END LOOP -------------------//
        wait_us(1000000);
    }
    return 1;
}
/**
 * @file IIRFilters.test.cpp
 * @version 1.0
 * @date 2020
 * @author Remy CHATEL & Patrik PAULINY
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Source code for "IIRFilter.test.h"
 * 
 * @see "IIRFilter.test.h"
 * 
 * # License
 *<b>(C) Copyright 2020 Patrik PAULINY</b> 
 *<b>(C) Copyright 2019 Remy CHATEL</b>
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

#include "IIRFilter.test.h"


int IIRFilterTest(){

    //--------------------- Initialise IMU----------------
    I2C i2c(D4, D5);
    i2c.frequency(400000);
    MPU9150 imu(&i2c);
    Timer SensorInterval;

    int mcount = 0; // Counts magnetometer samples 
    uint8_t MagRate = 80; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    uint8_t acc_scale = AFS_2G;
    uint8_t gyr_scale = GFS_250DPS;


   //IIR Low pass Filter Coeficients 
    float a1= -1.56101808;                  //-1.27963242;//-0.94280904;//-1.91119707;     
    float a2=  0.64135154;                  //0.47759225;//0.33333333;//0.91497583;  
    float b0= 0.02008337; //3Hz low pass    //0.04948996;//0.09763107;//0.00094469;//1Hz
    float b1= 0.04016673;                   //0.09897991;//0.19526215;//0.00188938;
    float b2= 0.02008337;                   //0.04948996;//0.09763107;//0.00094469;

    //Init Low-pass Buterworth IIR filter 
    Filters::IIRFilter IIR(a1, a2, b0, b1, b2);  
   
   //Variables to store previus Filter accumulators and delays 
    Matrix AccPrevius = Matrix::zeros(3,1);
    Matrix Delay1_previous = Matrix::zeros(3,1);
    Matrix Delay2_previous = Matrix::zeros(3,1);
    Matrix AccOUT_NOW = Matrix::zeros(3,1);

    // variables for data from the IMU 
    float val_mag[3], val_mag_filter[3];

    //--------------------- Start Communication with IMU and Calibrate ---------------------//
    printf("\n\r\n\r\n\r\n\r\n\r\n\r--------------------------------------\n\r");
    printf("Connection ok\n\r");
    uint8_t whoami = imu.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68 or 0x73\n\r");
    if (!imu.initIMU(acc_scale, gyr_scale)) {   // WHO_AM_I should be 0x68
        printf("Could not connect to MPU9150: \n\r");
        while(1) ; // Loop forever if communication doesn't happen
    }
    imu.recalibrateIMU(100, 100);
    float null_avg[3] = {0,0,0};
    imu.setAvgMag(null_avg);
    printf("IMU ok\n\r");

    // iterators for storing data to arrays (faster then instant send)
    int i = 0;
    int j = 0;
    int k = 0;
    int p = 0;
    int Send =0;


    while(1){    
        //--------------------- LOOP ---------------------//
        SensorInterval.reset();
        SensorInterval.start();
        imu.setAvgMag(null_avg);
        if(imu.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
            mcount++;
            if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
                imu.getMag(val_mag);  // Read the x/y/z adc values
                Matrix Mag_val(3,1,val_mag); // form a matrix so it can be used in the IIR filter 
                //----------------------IIR Butterworth LowPass filter-------------//
                AccOUT_NOW = IIR.Filter(Mag_val, AccPrevius, Delay1_previous, Delay2_previous); // IIR filtering  
                AccPrevius = IIR.getAccumulatorIN(); 
                Delay1_previous = IIR.getDelay1(); 
                Delay2_previous = IIR.getDelay2();
                mcount = 0;
                AccOUT_NOW.getCoef(val_mag_filter);
                printf("IIRFilter data, %f, %f, %f,\r\n", val_mag_filter[0], val_mag_filter[1], val_mag_filter[2]); //filtered data
                //printf("NoFilter data, %f,%f,%f,\r\n", val_mag[0], val_mag[1], val_mag[2]); // not filtered data 

                //---------- Accurate determination of Sampling Frequency --------
                //SensorInterval.stop();
                //Time=SensorInterval.read_ms();
                //printf(" \n ------TIME %f  TIME------- \n", Time);
                
            }
        }            
    }
}
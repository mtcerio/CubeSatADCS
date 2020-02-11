/**
 * @file   Filters.h
 * @defgroup FiltersGr Filters
 * @version 1.0
 * @date 2020
 * @author Remy CHATEL & Patrik PAULINY
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A library for filtering attitude quaternion and other signals
 * 
 * @details
 * # Description
 * A set of algorithm to filter the attitude quaternion of a spacecraft. It implements
 * a 7 state (quaternion and angular rates) Extended Kalman Filter and a Infinite Impulse Response (IIR) filter. 
 * 
 * It can be use to filter out noise in space applications for instance in Atitude
 * Determination and Control Systems. 
 * 
 * @see Filters::KalmanFilter
 * 
 * ## Working principle of the filter
 * Now that the attitude quaternion has been estimated with QuEst, it is very likely
 * that the result will have some noise and further filtering is required.
 * 
 * The Kalman filter is an algorithm that computes the best estimate of the state for
 * a time-varying process. Its predictor-corrector structure computes what the new state
 * should be according to the model and the elapsed time since the last update (prediction).
 * Then, it compares the prediction to the measured state in order to update the prediction
 * (correction) as shown in the figure below. The strength of the correction is
 * determined by the Kalman gain, which is computed using the different noise's covariances
 * 
 * This method avoids integrating since the fixed epoch, as the integration is done only
 * between two iterations of the filter while carrying the information of past measurements
 * to the next step to provide a better prediction.
 * 
 * ![Kalman Filter flowchart](https://upload.wikimedia.org/wikipedia/commons/thumb/a/a5/Basic_concept_of_Kalman_filtering.svg/640px-Basic_concept_of_Kalman_filtering.svg.png "Kalman Filter flowchart from Wikipedia")
 * 
 * ## Initialisation
 * The Kalman filter is initialized with knowledge of the spacecraft dynamic model and
 * statistical information about the process and the sensors (the covariance).
 * 
 * In the context of this project, the system state will be the combination of the
 * attitude quaternion and the angular rates. The resulting 7-state allows to propagate
 * the rotation of the spacecraft according to its dynamic model:
 * 
 * @f{align}{ 
 *     \dot{\mathbf{\omega}}   & = I^{-1} \left( -\mathbf{\omega}^\times I \mathbf{\omega} + \mathbf{u} + \mathbf{v} \right)\\
 *     \dot{\mathbf{\epsilon}} & = \frac{1}{2} \left( \eta \mathbf{1} + \mathbf{\epsilon}^\times \right) \mathbf{\omega}\\
 *     \dot{\eta}     & = \frac{1}{2} \mathbf{\epsilon}^T \mathbf{\omega}
 * @f}
 * 
 * Where @f$I@f$ is the inertia matrix of the satellite, @f$\eta@f$ the scalar part of
 * the attitude quaternion, @f$\epsilon@f$ the vector part of the quaternion, @f$\omega@f$
 * the angular rates, @f$u@f$ the control torque and @f$w@f$ the disturbance torque.
 * 
 * @see Filters::IIRFilter 
 * 
 * ## Working principle of the filter
 *
 * Almost every sensor is expected to produce readings that include some amount of noise that 
 * compromises its' readings to some extent. 
 *
 * IIR Filter is a structure much simplier than earlier described Kalman filter that can be used to easily 
 * implement low, high and band pass filters. These forms of digital signal procesing are commonly used and can 
 * dramaticaly improve  data aqusition. 
 * 
 * ##Initialisation 
 * IIR filter is initialised by IIR filter coeficients. Unfortunately coefitient generation is not part of this 
 * library yet. In order to generate IIR filter coefficients python scipy.signal library is recomended. The output 
 * is filtered stream of data if placed in a loop. Example can be seen in Detumbling_Test.cpp.  
 * 
 *More about IIR filter at: https://en.wikipedia.org/wiki/Infinite_impulse_response
 *
 * # Example code
 * 
 * @see Filters.test.cpp
 *
 * # Dependencies
 * This library depends on the "Matrix" library that can be found
 * here: https://github.com/RemyChatel/
 * 
 * In turn this library relies on <std::cmath> and <std::vector>
 * 
 * @attention This library uses float only (NOT double) and therefore
 * expect 6 to 7 significant figures
 * 
 * # References
 * - "Kalman Filtering and the Attitude Determination and Control Task",
 * by Hale, Vergez, Meerman and Hashida
 * - "Spacecraft Dynamic and Control: An introduction",
 * by A. de Ruiter, C. Damaren and J Forbes,
 * - "Fundamentals of Astrodynamics and Applications", by D. Vallado,
 * - Kalman Filter flowchart from Wikimedia, https://en.wikipedia.org/wiki/File:Basic_concept_of_Kalman_filtering.svg
 * 
 * # License
 * <b>(C) Copyright 2019 Remy CHATEL</b>
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
#ifndef FILTERS_H
#define FILTERS_H
#include "Matrix.h"

/**
 * @brief 
 * A namespace to hold the filtering processes
 * 
 * @namespace Filters
 * 
 * @see Filters
 */
namespace Filters{

/**
 * @ingroup FiltersGr
 * @brief
 * This class implements a 7-state (quaternion and angular rates)
 * Extended Kalman Filter for spacecraft atitude filtering.
 * 
 * @class Filters::KalmanFilter
 * 
 * @details
 * # Description
 * This class implements a 7-state (quaternion and angular rates)
 * Extended Kalman Filter for spacecraft atitude filtering.
 *
 * The initial inputs are the following:
 * - The inertia matrix of the satellite (3x3)
 * - The initial covariance matrix P (7x7)
 * - The process noise covariance matrix Q (7x7)
 * - The measurement noise covariance matrix R (7x7)
 * - The initial quaternion and angular rates
 * - The inertia matrix of the reaction wheels
 *
 * Once set up, the Kalman filter will predict the next quaternion using
 * an integration of how the system should evolve and the following inputs:
 * - The new measured quaternion
 * - The new measured angular rates
 * - The time ellapsed since last filter update
 * - The torques and angular rates of the control system of the spacecraft
 * 
 * @see Filters
 */
class KalmanFilter{
public:
// Constructors
    /**
     * @brief
     * Default constructor for the Kalman filter class
     */
    KalmanFilter();

    /**
     * @brief
     * Initialize the Kalman filter with the spacecraft and filter parameterss
     * @param I_sat_init    The inertia matrix of the satellite (in kg.m2) (3x3) Matrix
     * @param I_wheel_init  The inertia matrix of the reation wheels (in kg.m2) (3x3) Matrix
     * @param p_init        The initial covariance matrix (7x7) Matrix
     * @param kalman_q      The process noise covariance for the Kalman filter (7x7) Matrix
     * @param kalman_r      The sensor noise covariance for the Kalman filter (7x7) Matrix
     * @param q_init        The initial quaternion
     * @param w_init        The initial angular rates
     */
    KalmanFilter(Matrix I_sat_init, Matrix I_wheel_init, Matrix p_init, Matrix kalman_q, Matrix kalman_r, Matrix q_init, Matrix w_init);

    /**
     * @brief
     * Default destructor for the Kalman filter class
     */
    ~KalmanFilter(void);

// Getters and Setters
    /**
     * @brief
     * Fetched the predicted quaternion
     * @return The predicted quaternion
     */
    Matrix getQuaternion() const;

    /**
     * @brief
     * Fetched the predicted Angular Rate
     * @return The predicted Angular Rate
     */
    Matrix getAngularRate() const;

    /**
     * @brief
     * Fetched the predicted Covariance
     * @return The predicted Covariance
     */
    Matrix getCovariance() const;

// Filters
    /**
     * @brief
     * Filter the measured quaternion using a Kalman filter
     * @param q_measured      (@ step k)      Attitude quaternion measured from sensors (4x1) Matrix
     * @param w_measured      (@ step k)      Angular velocities in bf measured from sensors [rad/s] (3x1) Matrix
     * @param w_rw_prev       (@ step k-1)    Reaction wheel angular velocity [rad/s] at the previous time step (3x1) Matrix
     * @param T_bf_prev       (@ step k-1)    Total torque commanded to satellite in bf by external environment and magnetorquers (not reaction wheels!) [Nm] (3x1) Matrix  
     * @param T_rw_prev       (@ step k-1)    Torque commanded to satellite in bf by only reaction wheels [Nm] (3x1) Matrix
     * @param dt              (@ step k)      Simulation time step [sec].
     * @return The new predicted quaternion
     */
    Matrix filter(Matrix q_measured, Matrix w_measured, float dt, Matrix w_rw_prev, Matrix T_bf_prev, Matrix T_rw_prev);

private:
    Matrix I_sat;       /**< Inertia matrix of the spacecraft (in kg.m2) (3x3) Matrix */
    Matrix I_sat_inv;
    Matrix I_wheel;     /**< Inertia matrix of the reaction wheels (in kg.m2) (3x3) Matrix */

    Matrix q_predict;   /**< The predicted quaternion at step k (4x1) Matrix */
    Matrix w_predict;   /**< The predicted angular rates at step k (3x1) Matrix */
    Matrix p_predict;   /**<  The predicted covariance matrix at step k (7x7) Matrix */

    Matrix _kalman_q;   /**< Process noise covariance */
    Matrix _kalman_r;   /**< Sensor noise covariance */

}; // class KalmanFilter



class IIRFilter{
public: 
// Constructors
    /**
     * @brief
     * Default constructor for the Infinite Impulese Response filter class
     */
    IIRFilter();

    /**
     * @brief
     * Initialize the IIR filter with the IIR Filter coeficients
     * @param A1   The feedback gain coefficient 
     * @param A2   Second feedback coefficient  
     * @param B0   First feed forward coefficient 
     * @param B1   Second feed forward coefficient 
     * @param B2   Third  feed forward coefficient 
     * @param w_init        The initial angular rates
     */
    IIRFilter(float A1, float A2, float B0, float B1, float B2);

    void ResetFilter();

    ~IIRFilter(void);

    /**
     * @brief
     * Fetched the input accumulator value 
     * @return the input accumulator value 
     */
    Matrix getAccumulatorIN() const;
    
    /**
     * @brief
     * Fetched the output of the IIR Filter 
     * @return the output of the IIR Filter
     */
    Matrix getAccumulatorOUT() const;
    
    /**
     * @brief
     * Fetched the value of input accumulator one time step ago 
     * @return the value of input accumulator one time step ago 
     */
    Matrix getDelay1() const;
    
    /**
     * @brief
     * Fetched the value just before second accumulator 
     * @return the value just before second accumulator 
     */
    Matrix getDelay2() const;

    /**
     * @brief
     * Perform the IIR Filter and calculate new values for input accumulator, output accumulator, delay1 and delay2. 
     * @return The value of output accumulator 
     */
    Matrix Filter(Matrix InPut, Matrix AccumulatorIN_previous,  Matrix Delay1_previous, Matrix Delay2_previous);

private: 
    Matrix AccumulatorIN; //Input accumulator 
    Matrix AccumulatorOUT; //Output accumulator 
    Matrix Dat_Input; // Input to the IIR filter 
    Matrix Delay1; // Input acumulator after one time step 
    Matrix Delay2; // Input acumulator after two time steps 

    float _A1;
    float _A2;
    float _B0;
    float _B1;
    float _B2;
};
}; // namespace Filters
#endif // FILTERS_H
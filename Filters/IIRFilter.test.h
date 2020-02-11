/**
 * @file IIRFilter.test.h
 * @version 1.0
 * @date 2020
 * @author Remy CHATEL & Patrik PAULINY
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * Test framework library for the IIRFilter class
 * 
 * @details
 * # Description
 * This library provides a test function for the IIRFilter class and also
 * serves as an example program for the module.
 *
 * 
 * @see Filters.h
 * 
 * # Dependencies and data type
 * This library depends on:
 * - the Mbed framework (https://www.mbed.com/en/)
 * - <std::cmath> in order to perform cos, sin and sqrt operations.
 * - <std::vector> for the matrix algebra
 * - "Matrix.h" for matrices implementation
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
 * 
 * # License
 * <b>(C) Copyright 2020 Patrik PAULINY</b>
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

#ifndef IIRFILTERS_TEST_H
#define IIRFILTERS_TEST_H
#include "mbed.h"
#include "Matrix.h"
#include "MPU9150.h"
#include "Filters.h"
/**
 * @brief
 * Test of the IIRFilter class 
 * 
 * Tests the IIRFilters module by making measurements of magnetic field using MPU9150 
 * and printing them to the serial monitor. Filtered and not filteraed data can be stored 
 * and compared to provide also qualitative feedback. 
 * 
 */
int IIRFilterTest();

#endif
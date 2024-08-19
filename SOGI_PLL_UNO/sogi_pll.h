/* This program is distributed under the terms of the GNU Affero General Public License v3.0.
 *
 * The GNU Affero General Public License is a copyleft license that requires any modified
 * version of the program to be made available under the same license. The license also
 * requires that anyone who distributes the program, or a modified version of it, must make
 * the corresponding source code available under the same license. This ensures that users
 * of the program have the freedom to study, modify, and distribute the code as they see fit.
 *
 * For more information about the GNU Affero General Public License v3.0, please see
 * https://www.gnu.org/licenses/agpl-3.0.en.html
 * https://faultyproject.es
 */

#ifndef _SOGI_PLL_H_
#define _SOGI_PLL_H_
#include <math.h>

void sogi_pll_start(float k, float wn, float kp, float ki, float ts);
float sogi_pll_update(float u_in);
float sogi_pll_get_freq();
float sogi_pll_get_ampli();

#endif
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

#include "sogi_pll.h"

const float twoPI = 6.283185307179586476925286766559;			//2*PI
const float one_twoPI = 0.15915494309189533576888376337251;	//1/(2*PI)

//SOGI_PLL Parameter
float ts;		    //Sample time
float k_sogi;   //SOGI k parameter
float wn_pll;   //Central frequency
float kp_pll;	  //PI Proportional Gain
float ki_pll;	  //PI Integral Gain
float b_pll[3];
float c_pll[3];
float a_pll[3];

//SOGI_PLL Signals
float ug_pll[3];	  //Input signal
float yd_pll[3];	  //Direct SOGI output
float yq_pll[3];	  //Quadrature SOGI output
float ud_pll[2];	  //Park Transformation Output (d)-q
float pi_pll[2];	  //PI Out
float theta_pll[2];	//Input signal phase
float omega_pll[2]; //Input signal frecuency (rad/s)
float freq_pll;		  //Input signal frecuency (Hz)
float ampli_pll;    //Input signal amplitude

void sogi_pll_start(float k_0, float wn_0, float kp_0, float ki_0, float ts_0){
  k_sogi = k_0;
  wn_pll = wn_0;
  kp_pll = kp_0;
  ki_pll = ki_0;
  ts = ts_0;

  float x = 2.0F * k_sogi * ts * wn_pll;
  float y = wn_pll * wn_pll * ts * ts;

  a_pll[1] = (2.0F*(4.0F-y))/(x+y+4.0F);
  a_pll[2] = (x-y-4.0F)/(x+y+4.0F);
  b_pll[0] = x/(x+y+4.0F);
  c_pll[0] = y/(x+y+4.0F);

  ug_pll[0] = 0.0;
  ug_pll[1] = 0.0;
  ug_pll[2] = 0.0;
  yd_pll[0] = 0.0;
  yd_pll[1] = 0.0;
  yd_pll[2] = 0.0;
  yq_pll[0] = 0.0;
  yq_pll[1] = 0.0;
  yq_pll[2] = 0.0;
  ud_pll[0] = 0.0;
  ud_pll[1] = 0.0;
  pi_pll[0] = 0.0;
  pi_pll[1] = 0.0;
  theta_pll[0] = 0.0;
  theta_pll[1] = 0.0;
  omega_pll[0] = 0.0;
  omega_pll[1] = 0.0;
}

float sogi_pll_update(float u_in){
  ug_pll[0] = u_in;
  //Calcula salida del SOGI
  yd_pll[0] = a_pll[1]*yd_pll[1] + a_pll[2]*yd_pll[2] + b_pll[0]*(ug_pll[0] - ug_pll[2]);
  yq_pll[0] = a_pll[1]*yq_pll[1] + a_pll[2]*yq_pll[2] + k_sogi*c_pll[0]*(ug_pll[0] + 2.0F*ug_pll[1] + ug_pll[2]);

  ampli_pll = (M_SQRT1_2)*sqrtf(yd_pll[0]*yd_pll[0] + yq_pll[0]*yq_pll[0]);

  //Transformada park
  ud_pll[0] = yd_pll[0]*cosf(theta_pll[0]) + yq_pll[0]*sinf(theta_pll[0]);

  //PI_pll
  pi_pll[0] = pi_pll[1] + kp_pll*ud_pll[0] + (ki_pll*ts-kp_pll)*ud_pll[1];

  //Frecuencia de red
  omega_pll[0] = wn_pll + pi_pll[0];

  //Fase de red (integrador)
  theta_pll[0] = theta_pll[1] + ts*omega_pll[1];

  if(theta_pll[0] >= twoPI){
    theta_pll[0] -= twoPI;
  }

  freq_pll = omega_pll[0]*one_twoPI;

  //Guarda estado del PLL
  omega_pll[1] = omega_pll[0];
  theta_pll[1] = theta_pll[0];
  pi_pll[1] = pi_pll[0];
  ud_pll[1] = ud_pll[0];
  yd_pll[2] = yd_pll[1];
  yd_pll[1] = yd_pll[0];
  yq_pll[2] = yq_pll[1];
  yq_pll[1] = yq_pll[0];
  ug_pll[2] = ug_pll[1];
  ug_pll[1] = ug_pll[0];

  return theta_pll[0];
}

float sogi_pll_get_freq(){
  return freq_pll;
}
float sogi_pll_get_ampli(){
  return ampli_pll;
}
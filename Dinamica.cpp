#include "Dinamica.h"
#include <cmath>

Dinamica::Dinamica(){
    MZ_tib = 1.2;
    XX_tib = 0.354;
    atrito = 1.5;
    g = 9.81;
}

DinamicaDados Dinamica::getDinamica(double pos, double vel){
    DinamicaDados resultados;
    resultados.M =  XX_tib;
    resultados.C = atrito * vel;
    resultados.G = MZ_tib * g * sin(pos);

    return resultados;
}

double Dinamica::calcularAmortecimentoOtimo(double Kr_atual, double q2_lqr, double R_lqr){
    double termo_C2 = atrito * atrito;
    double termo_M2_QR = (XX_tib * XX_tib) * (q2_lqr / R_lqr);
    double termo_2MK = 2.0 * XX_tib * Kr_atual;
    double interior_raiz = termo_C2 + termo_M2_QR + termo_2MK;

    return -atrito + sqrt(interior_raiz);
}
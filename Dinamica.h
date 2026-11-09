#pragma once

struct DinamicaDados {
    double M;
    double C;
    double G;
};

class Dinamica {
    private:
        double MZ_tib;
        double XX_tib;
        double atrito;

    public:
        double g;
        Dinamica();
        DinamicaDados getDinamica(double pos, double vel);
        double calcularAmortecimentoOtimo(double Kr_atual, double q2_lqr, double R_lqr);
};
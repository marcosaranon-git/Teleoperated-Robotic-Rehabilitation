#include <iostream>
#include <cmath>
#include "Dinamica.h"

int main() {
    // 1. Instanciamos a sua classe
    Dinamica perna;

    // 2. Simulamos uma leitura do encoder do exoesqueleto
    // Vamos testar com o joelho a 30 graus e movendo a 1 rad/s.
    // Lembrando: a função sin() do C++ exige radianos!
    double pos_graus = 30.0;
    double pos_rad = pos_graus * (M_PI / 180.0); 
    double vel_rad_s = 1.0;

    // 3. Chamamos a função que você criou e guardamos a struct retornada
    DinamicaDados matrizes = perna.getDinamica(pos_rad, vel_rad_s);

    double Kr_terapeuta = 50.0; 

    // Pede para a classe calcular o Br ótimo (ex: q2=10.0, R=1.0)
    double Br_otimo = perna.calcularAmortecimentoOtimo(Kr_terapeuta, 10.0, 1.0);

    // 4. Imprimimos os resultados no terminal
    std::cout << "--- Teste da Dinamica do Exoesqueleto ---" << std::endl;
    std::cout << "Posicao: " << pos_graus << " graus (" << pos_rad << " rad)" << std::endl;
    std::cout << "Velocidade: " << vel_rad_s << " rad/s" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "M (Inercia)   : " << matrizes.M << std::endl;
    std::cout << "C (Atrito)    : " << matrizes.C << std::endl;
    std::cout << "G (Gravidade) : " << matrizes.G << std::endl;
    std::cout << "Br (Amortecimento ótimo): " << Br_otimo << std::endl;

    return 0;
}
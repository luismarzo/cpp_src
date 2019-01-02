#include "flex.h"



float flex_simple(float _f, float _l, float _i, float _e){
std::cout<<"Bending of a bridge, parameters are:"<< std::endl;
std::cout<<"F [N]:"<<_f<< std::endl;
std::cout<<"L [m]:"<<_l<< std::endl;
std::cout<<"Izz [m⁴]:"<<_i<< std::endl;
std::cout<<"E [N/m²]:"<<_e<< std::endl;

std::cout<<"            F           "<< std::endl;
std::cout<<"            |           "<< std::endl;
std::cout<<"            |           "<< std::endl;
std::cout<<"            |           "<< std::endl;
std::cout<<"A-----------C-----------B"<< std::endl;


float uy=(_f*_l*_l*_l)/(48*_e*_i);
std::cout<<"The bending in the y axis is "<<uy<< std::endl;


}


float flex_full(float _f, float _l, float _i, float _e){
std::cout<<"Bending of a bridge, parameters are:"<< std::endl;
std::cout<<"F [N]:"<<_f<< std::endl;
std::cout<<"L [m]:"<<_l<< std::endl;
std::cout<<"Izz [m⁴]:"<<_i<< std::endl;
std::cout<<"E [N/m²]:"<<_e<< std::endl;

std::cout<<"            F           "<< std::endl;
std::cout<<"||||||||||||||||||||||||"<< std::endl;
std::cout<<"||||||||||||||||||||||||"<< std::endl;
std::cout<<"||||||||||||||||||||||||"<< std::endl;
std::cout<<"A-----------C-----------B"<< std::endl;

float uy=(5*_f*_l*_l*_l*_l)/(384*_e*_i);
return uy;
std::cout<<"The bending in the y axis is "<<uy<< std::endl;


}

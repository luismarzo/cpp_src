#include <iostream>
#include "flex.cpp"


int main() 
{
   float f=1;
   float l=2;
   float i=3;
   float e=4;
   float res_simple=flex_simple(f,l,i,e);
   float res_full=flex_full(f,l,i,e);
   
   std::cout<<"The bending in the y axis in the 'C' point and simple force case is "<<res_simple<< std::endl;
   std::cout<<"The bending in the y axis in the 'C' point and full force case is "<<res_full<< std::endl;
   return 0;
}

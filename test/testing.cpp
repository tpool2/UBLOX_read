#include <iostream>
#include <cstdint>
#include <cmath>

int endian()
{
    int i = 1;
    char *p = (char *)&i;
    if(p[0]==1)
    {
        return 0; //Little endian
    }
    else
    {
        return 1; //Big endian
    }
}



int main(int argc, char* argv[])
{
    if(endian()==0)
    {
        std::cout<<"Little endian"<<std::endl;
    }
    else
    {
        std::cout<<"Big endian"<<std::endl;
    }
}
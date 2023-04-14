#include <iostream>

#include "Arr.h"




int main()
{
    tArr s1 = {};
    

    InitArr(&s1);

    for (int i = 0 ; i < 10 ; ++i)
    {
        PushBack(&s1, i);
    }
   

    ReleaseArr(&s1);

    
   

    // 초기값



   
}

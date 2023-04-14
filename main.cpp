#include <iostream>

#include "Arr.h"




int main()
{
    tArr s1 = {};
    

    InitArr(&s1);

    for (int i = 0 ; i < 10 ; ++i)
    {
        PushBack(&s1, i);
        //추가한거 보낼거임
    }
   

    ReleaseArr(&s1);

    
   

    // 초기값



   
}

#include "Arr.h"

#include <iostream>

void InitArr(tArr* _pArr)
{
    _pArr->pInt = (int*)malloc(sizeof(int)*2);
    _pArr->iCount = 0;
    _pArr->iMaxCount = 2;

}

void ReleaseArr(tArr* _pArr)
{
    free(_pArr->pInt);
    _pArr->iCount = 0;
    _pArr->iMaxCount = 0;
}

void PushBack(tArr* _pArr, int _iData)
{
    // 힙 영역에 할당한 공간이 다참
    if (_pArr -> iMaxCount <= _pArr ->iCount)
    {
        //재할당
        Reallocate(_pArr);
    }

    // 데이터 추가
    _pArr->pInt[_pArr->iCount] = _iData;

    
    
}

void Reallocate(tArr* _pArr)
{
    int* pNew = (int*)malloc(_pArr->iMaxCount * 2 * sizeof(int));

    for (int i = 0 ; i < _pArr->iCount ; ++i)
    {
        pNew[i] = _pArr -> pInt[i];
    }
    
    free(_pArr->pInt);

    _pArr->pInt = pNew;

    _pArr->iMaxCount *= 2;
}


typedef struct _tagArr
{
    int* pInt;
    int iCount;
    int iMaxCount;

}tArr;

//초기화함수

void InitArr(tArr* _pArr);

// 메모리 해제
void ReleaseArr(tArr* _pArr);

// 데이터 추가 함수

void PushBack(tArr* _pArr, int _iData);

// 공간 추가 확장

void Reallocate(tArr* _pArr);

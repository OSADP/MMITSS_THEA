//**********************************************************************************
//
// © 2015 Arizona Board of Regents on behalf of the University of Arizona with rights
//       granted for USDOT OSADP distribution with the Apache 2.0 open source license.
//
//**********************************************************************************

#pragma once
//  Solution from : TestConfig
#include <sstream>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "facilityLayer.hpp"

using namespace std;

/*
*** Array has n elements, find the index of value equal data.
*** Return the index used by an array directly, which means should be (real count -1)
*/
template <class T>
int FindIndexArray(T* Array, int n, T data)
{
    int index = -1;
    for (int i = 0; i < n; i++)
    {
        if (data == Array[i])
        {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        ITSAPP_WRN("***Error*** Data is not in the Array !");
        return index;
    }
    return index;
}

template <class T>
int FindIndexArray(const std::vector<T>& Array, int n, T data)
{
    int index = -1;
    for (int i = 0; i < n; i++)
    {
        if (data == Array[i])
        {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        ITSAPP_WRN("***Error*** Data is not in the Array !");
        return index;
    }
    return index;
}

/*
*** "Array" has "n" elements, find the "nTh" index of value equal "data"
*/
template <class T>
int FindIndexArray(T* Array, int n, T data, int nTh)
{
    int index = -1;
    int FoundTimes = 0;
    for (int i = 0; i < n; i++)
    {
        if (data == Array[i])
        {
            FoundTimes++;
            if (FoundTimes == nTh)
            {
                index = i;
                break;
            }
        }
    }
    return index;
}

template <class T>
int FindIndexArray(const std::vector<T>& Array, int n, T data, int nTh)
{
    int index = -1;
    int FoundTimes = 0;
    for (int i = 0; i < n; i++)
    {
        if (data == Array[i])
        {
            FoundTimes++;
            if (FoundTimes == nTh)
            {
                index = i;
                break;
            }
        }
    }
    return index;
}

/*
*** Summation of an array from StartIdx till EndIdx, EndIdx not included.
*/
template <class T>
T SumArray(T* Array, int n, int StartIdx, int EndIdx)
{
    T SumValue = 0;
    if ((StartIdx > (n - 1)) || (EndIdx > (n - 1)) || (StartIdx > EndIdx))
    {
        ITSAPP_WRN("There is problem with the index!!!");
        exit(1);
    }
    else
    {
        for (int i = StartIdx; i < EndIdx; i++)
        {
            SumValue += Array[i];
        }
    }
    return SumValue;
}

template <class T>
void PrintArray(T Array, int n)
{
    std::stringstream s;
    for (int i = 0; i < n; i++)
    {
        s << Array[i] << "\t";
    }
    ITSAPP_LOG("%s", s.str().c_str());
}

template <class T>
void PrintArray(T* Array, int n)
{
    std::stringstream s;
    for (int i = 0; i < n; i++)
    {
        s << Array[i] << "\t";
    }
    ITSAPP_LOG("%s", s.str().c_str());
}

// void GeneratePhaseArray(int StartPhase, int Number, int Cycle ,int *PhaseSeq);
void GeneratePhaseArray(int startphase, int number, int cycle, int* phase_seq, int* ResultSeq);
std::vector<int> GeneratePhaseArray(
    int StartPhase, const std::vector<int>& PhaseSeq, int N, int TotalNo, int NoRepeat = 0);

int BarrierPhaseIsMissing(int ReqPhase, int* InitPhase, int size);
int BarrierPhaseIsMissing2(int ReqPhase, int* InitPhase, int size);
int BarrierPhaseIsMissing(int ReqPhase, vector<int> Phase_vc);

int AllSameElementOfVector(vector<int> EV_Phase_vc);
void bubbleSort(int arr[], int n);
int removeDuplicates(int a[], int array_size);
void selectionSort(int a[], int size);
void binary(int number);
void xTimeStamp(char* pc_TimeStamp_);

int msleep(unsigned long milisec);
int FindCycleIdx(double timeStamp[3], double timer); // 3 for cycles

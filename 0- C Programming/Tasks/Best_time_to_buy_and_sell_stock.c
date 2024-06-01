#include <stdio.h>

/*
    Author: Salma Faragalla
    Description: https://leetcode.com/problems/best-time-to-buy-and-sell-stock/description/
*/

int maxProfit(int *arr, int arrSize);
   
int main()
{
    /*
    int arr[6]={7,1,5,3,6,4};
    int arrSize=6;
    */

    /*
    int arr[5]={3,3,3,3,3};
    int arrSize=5;
    */

    /*
    int arr[5]={7,6,5,4,3};
    int arrSize=5;
    */

    int arr[5] = {2, 6, 1, 4, 3};
    int arrSize = 5;

    int max_profit = maxProfit (arr,arrSize);
    printf("The maximum profit = %d", max_profit);
}


int maxProfit(int *arr, int arrSize)
{
    int arrCounter = 0;
    int min = arr[0];
    int profit = 0;

    for (arrCounter = 1; arrCounter < arrSize; ++arrCounter)
    {
        if (arr[arrCounter] < min)
        {
            min = arr[arrCounter];
        }

        else if (arr[arrCounter] - min > profit)
        {
            profit = arr[arrCounter] - min;
        }
    }
    return profit;
}
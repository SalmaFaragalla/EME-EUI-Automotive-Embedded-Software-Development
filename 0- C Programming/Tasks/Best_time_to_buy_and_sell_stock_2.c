#include <stdio.h>

/*
    Author: Salma Faragalla
    Description: https://leetcode.com/problems/best-time-to-buy-and-sell-stock-ii/description/
*/

int maxProfit(int *arr, int arrSize);
   
int main()
{
    
    int arr[6]={7,1,5,3,6,4};
    int arrSize=6;
    

    /*
    int arr[5]={1,2,3,4,5};
    int arrSize=5;
    */

    /*
    int arr[5]={7,6,4,3,1};
    int arrSize=5;
    */

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

        else
        {
            profit += arr[arrCounter] - min;
            if (arrCounter + 1 < arrSize)
            {
                if (arr[arrCounter] < arr[arrCounter + 1])
                {
                    min = arr[arrCounter];
                }
                else
                {
                    min = arr[arrCounter + 1];
                }
            }
        }
    }
    return profit;
}
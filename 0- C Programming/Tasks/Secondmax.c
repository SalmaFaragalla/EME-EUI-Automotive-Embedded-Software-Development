/*
    Author: Salma Faragalla
    Description: - Function to find the second maximum value in an array using sorting
                 - Function to find the second maximum value in an array without sorting

*/

#include <stdio.h>

#define ARR_SIZE 7

void Swap(int *num1, int *num2);
int Second_max_sorting(int *arr, int arr_size);
int Second_max_without_sorting(int *arr, int arr_size);

static int flag = 0;

int main()
{
    int secondMax = 0;
    int arr[ARR_SIZE];
    /* int arr[5] = {8, 20, 17, 9, 20}; */
    /* int arr[3]= {3,3,3};*/
    printf("Please enter %d numbers.\n", ARR_SIZE);
    for (int arr_counter = 0; arr_counter < ARR_SIZE; ++arr_counter)
    {
        printf("Please enter number %d: ", arr_counter + 1);
        scanf("%d", &arr[arr_counter]);
    }

    // secondMax=Second_max_sorting(arr,arr_size);
    secondMax = Second_max_without_sorting(arr, ARR_SIZE);
    if (1 == flag)
    {
        printf("\nNo second maximum , the whole array is the same value.");
    }
    else
    {
        printf("\nThe second max = %d", secondMax);
    }
}

void Swap(int *num1, int *num2)
{
    *num1 = *num1 ^ *num2;
    *num2 = *num1 ^ *num2;
    *num1 = *num1 ^ *num2;
}
int Second_max_sorting(int *arr, int arr_size)
{
    int arr_counter = 0;
    int swap_counter = 0;

    if (arr_size == 1)
    {
        flag = 1;
        return arr[0];
    }

    /*Bubble Sort*/
    for (arr_counter = 0; arr_counter < 2; ++arr_counter)
    {
        for (swap_counter = 0; swap_counter < arr_size - 1; ++swap_counter)
        {
            if (arr[swap_counter] > arr[swap_counter + 1])
            {
                Swap(&arr[swap_counter], &arr[swap_counter + 1]);
            }
        }
    }

    if (arr[arr_size - 1] == arr[arr_size - 2])
    {
        flag = 1;
    }
    else
    {
        for (arr_counter = 0; arr_counter < arr_size; ++arr_counter)
        {
            printf(" %d ", arr[arr_counter]);
        }
    }
    return arr[arr_size - 2];
}

int Second_max_without_sorting(int *arr, int arr_size)
{
    int max = arr[0];
    int second_max = arr[1];
    int counter = 0;

    if (arr_size == 1)
    {
        flag = 1;
        return arr[0];
    }
    if (second_max > max)
    {
        second_max = arr[0];
        max = arr[1];
    }

    for (counter = 2; counter < arr_size; ++counter)
    {
        if (arr[counter] > max)
        {
            second_max = max;
            max = arr[counter];
        }
        else if (arr[counter] > second_max && arr[counter] < max)
        {
            second_max = arr[counter];
        }
    }

    if (max == second_max)
    {
        flag = 1;
    }
    return second_max;
}

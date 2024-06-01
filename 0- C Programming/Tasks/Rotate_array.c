/*
    Author: Salma Faragalla
    Description: Given an integer array , rotate the array to the right by k steps, when k is non-negative 
    and rotate the array to the left by k steps when k is negative
*/
#include <stdio.h>
#include <stdlib.h>

void rotate_left (int * arr , int arrSize,int k);
void rotate_right(int * arr, int arrSize,int k);
void print_array (int * arr, int arrSize);
void swap (int *num1 ,int *num2);


int main ()
{
    int k=-2;

    printf ("Enter the number of rotation k = ");
    scanf ("%d",&k);

    k= k%6;
    printf ("k= %d\n",k);

    int arr [6]={1,2,3,4,5,6};

    if (k>=0)
    {
        rotate_right (arr,6,k);
        print_array(arr,6);

    }
    else if (k<0)
    {
        rotate_left(arr,6,abs(k));
        print_array(arr,6);
    }
        
}



void print_array (int * arr, int arrSize)
{
    for (int i=0 ; i<arrSize ;++i)
    {
        printf ("%d ",arr[i]);
    }
}

void swap(int *num1 ,int *num2)
{
    *num1 = *num1 ^ *num2;
    *num2 = *num1 ^ *num2;
    *num1 = *num1 ^ *num2;

}
 
void rotate_right (int * arr, int arrSize ,int k)
{
    /*Reverse all*/
   for (int i =0 ; i<arrSize/2 ; ++i )
   {
        swap(&arr[i],&arr[arrSize-1-i]);
   }

    /*Reverse from 0 to k */
    for (int i =0 ; i<k/2 ; ++i )
   {
        swap(&arr[i],&arr[k-1-i]);
   }

    /*Reverse from k to rest*/
   for (int i=k ; i< k+ (arrSize-k)/2  ; ++i )
   {
        swap(&arr[i],&arr[arrSize-1-(i-k)]);
   }
}

void rotate_left (int * arr , int arrSize,int k)
{
    /*Reverse all*/
   for (int i =0 ; i<arrSize/2 ; ++i )
   {
        swap(&arr[i],&arr[arrSize-1-i]);
   }

    /*Reverse from 0 to arrsize-k */
    for (int i =0 ; i<(arrSize-k)/2 ; ++i )
   {
        swap(&arr[i],&arr[arrSize-k-1-i]);
   }

    /*Reverse from arrSize-k to rest*/
   for (int i =arrSize-k ; i<(arrSize-k) + k/2  ; ++i )
   {

        swap(&arr[i],&arr[arrSize-1-(i-(arrSize-k))]);
   }


}



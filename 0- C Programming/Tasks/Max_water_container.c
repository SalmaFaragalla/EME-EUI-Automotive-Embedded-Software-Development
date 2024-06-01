/*
    Author: Salma Faragalla
    Description: https://leetcode.com/problems/container-with-most-water/ 
*/

#include <stdio.h>
int max_container (int * height , int n);

int main ()
{
    int n=9;
    int height [9]= {1,8,6,2,5,4,8,3,7};
    printf ("%d",max_container(height,n));
}

int max_container (int * height , int n)
{
    int* start_ptr = &height[0];
    int* end_ptr = &height[n-1];
    int min=0;
    int max=0;

    while (start_ptr != end_ptr)
    {
        if ( *start_ptr < *end_ptr )
        {
            min = *start_ptr ;
        }
        else
        {
            min = *end_ptr ;
        }

        if ( (min*((end_ptr-start_ptr))) > max)
        {
            max=min*((end_ptr-start_ptr));
        }

        if ( min == *start_ptr )
        {
           ++start_ptr;
        }
        else
        {
            end_ptr-- ;
        }

    }

    return max;
}

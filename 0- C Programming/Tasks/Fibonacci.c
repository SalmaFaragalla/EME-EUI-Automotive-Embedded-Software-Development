/*
    Author: Salma Faragalla
    Description: Function to find and print Nth Fibonacci Numbers
*/

#include <stdio.h>

void Fibonacci (int n);
int main ()
{
    int n=0;
    scanf("%d",&n);
    while (n<=0)
    {
        printf("Please enter a positive value greater than zero : ");
        scanf("%d",&n);   
    }
    Fibonacci (n);

}

void Fibonacci (int n)
{
    int counter=0;
    int num1,num2,sum;
    for (counter=0 ; counter<n ; ++counter)
    {
        if (0==counter)
        {
            num1=0;
            printf(" %d ",num1);

        }
        else if (1==counter)
        {
            num2=1;
            printf(" %d ",num2);
        }
        else
        {  
            sum=num1+num2;
            printf(" %d ",sum);
            num1=num2;
            num2=sum;
        }

    }
 

}
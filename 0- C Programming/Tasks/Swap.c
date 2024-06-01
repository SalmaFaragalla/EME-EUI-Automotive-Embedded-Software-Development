/*
    Author: Salma Faragalla
    Description: Function to swap to integers using 4 methods
*/

#include <stdio.h>

void Swap_temp(int *num1 , int *num2);
void Swap_add (int *num1 , int *num2);
void Swap_mul (int *num1 , int *num2);
void Swap_xor (int *num1 , int *num2);

int main ()
{
    int num1,num2;
    scanf("%d %d",&num1,&num2);
    //Swap_temp(&num1,&num2);
    //Swap_add(&num1 ,&num2);
    Swap_mul(&num1 ,&num2);
    //Swap_xor (&num1 ,&num2);

    printf("\n%d\n%d",num1,num2);

}

void Swap_temp(int *num1 , int *num2)
{
    int temp=0;
    temp = *num1;
    *num1=*num2;
    *num2=temp;
}

void Swap_add (int *num1 , int *num2)
{
    *num1 = *num1 + *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);
    *num2 = *num1 - *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);
    *num1 = *num1 - *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);
}

void Swap_mul (int *num1 ,int *num2)
{

    if (0==*num2)
    {
        *num2=*num1;
        *num1=0;
    }
    else if (0==*num1)
    {
        *num1=*num2;
        *num2=0;

    }
    else 
    {
        *num1 = *num1 * *num2;
        printf ("num1= %d num2= %d \n",*num1,*num2);
        *num2 = *num1 / *num2;
        printf ("num1= %d num2= %d \n",*num1,*num2);
        *num1 = *num1 / *num2;
        printf ("num1= %d num2= %d \n",*num1,*num2);

    }


}

void Swap_xor (int *num1 ,int *num2)
{
    *num1 = *num1 ^ *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);
    *num2 = *num1 ^ *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);
    *num1 = *num1 ^ *num2;
    printf ("num1= %d num2= %d \n",*num1,*num2);

}

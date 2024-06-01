/*
    Author: Salma Faragalla
    Description: Calculator program in c using function pointers
*/

#include <stdio.h>
#include <stdlib.h>

float add (float n1 ,float n2);
float subtract (float n1 ,float n2);
float multiply (float n1 ,float n2);
float divide (float n1 ,float n2);
float power (float n1 , int exponent);
int factorial (int n1);
void get_one_operand (int *n1);
void get_two_operand (float *n1, float*n2);
void get_two_operand_power (float *n1, int*n2);

enum operations_names
{
    addition ,
    subtraction,
    multiplication,
    division

};
int main ()
{
    char continue_calc='Y';
    int operationID=0;
    float n1,n2;
    int n_int;

    float (*operations_ptrs[4]) (float,float) = {add,subtract,multiply,divide};
    while ('Y'==continue_calc)
    {
        printf ("Please choose from the following operations : \n");
        printf ("1)Addition\n");
        printf ("2)Subtraction\n");
        printf ("3)Multiplication\n");
        printf ("4)Division\n");
        printf ("5)Power\n");
        printf ("6)Factorial\n");

        printf ("Please enter the operation ID : ");
        scanf ("%d",&operationID);

        while (operationID<1 || operationID>6)
        {
            printf ("Invalid operation ID\n");
            printf ("Please enter the operation ID : ");
            scanf ("%d",&operationID);
        }

        switch (operationID)
        {
            case 1:
                get_two_operand(&n1,&n2);
                printf("The result = %f",operations_ptrs[addition](n1,n2));
                break;
            case 2:
                get_two_operand(&n1,&n2);
                printf("The result = %f",operations_ptrs[subtraction](n1,n2));
                break;
            case 3:
                get_two_operand(&n1,&n2);
                printf("The result = %f",operations_ptrs[multiplication](n1,n2));
                break;
            case 4:
                get_two_operand(&n1,&n2);
                
                while (0==n2)
                {
                    printf("Can't divide by zero , Please enter valid values.\n");
                    get_two_operand(&n1,&n2);
                }
                printf("The result = %f",operations_ptrs[division](n1,n2));            
                break;
            case 5:
                get_two_operand_power(&n1,&n_int);
                printf("The result = %f",power(n1,n_int));                        
                break;
            case 6:
                get_one_operand(&n_int);
                printf("The result = %d",factorial(n_int));
                break;
        
        }
        printf("\n");

        printf ("Do you want to continue ? (Y/N) : ");
        scanf (" %c",&continue_calc);
        while (continue_calc != 'Y' && continue_calc != 'N')
        {
            printf("Please enter either Y or N ");
            scanf (" %c",&continue_calc);
        }
        printf("\n");
            
    }

    return 0;
    
}

float add (float n1 ,float n2)
{
    return n1+n2;
}
float subtract (float n1 ,float n2)
{
    return n1-n2;
}
float multiply (float n1 ,float n2)
{
    return n1*n2;
}
float divide (float n1 ,float n2)
{
    return n1/n2;
}
float power (float n1 , int exponent)
{
    if (0 == exponent) return 1;
    
    else if (exponent > 0)
    {
        float result=n1;
        for (float i=2 ; i<= exponent ; ++i)
        {
            result=result*n1;
        }
        return result;
    }

    else
    {
        float result=1/n1;
        for (float i=2 ; i<= abs(exponent) ; ++i)
        {
            result=result*(1/n1);
        }
        return result;

    }
 
}

int factorial (int n1)
{
    int result=n1;
    for (int i=n1 ; i>1 ; --i)
    {
        --n1;
        result=result*n1;
    }
    return result;
}

void get_one_operand (int *n1)
{
    printf ("Please enter the operand : ");
    scanf ("%d",&(*n1));
}
void get_two_operand (float *n1, float*n2)
{
    printf ("Please enter operand 1 : ");
    scanf("%f",&(*n1));

    printf ("Please enter operand 2 : ");
    scanf("%f",&(*n2));

}
void get_two_operand_power (float *n1, int* n2)
{
    printf ("Please enter operand 1 : ");
    scanf("%f",&(*n1));

    printf ("Please enter operand 2 : ");
    scanf("%d",&(*n2));

}

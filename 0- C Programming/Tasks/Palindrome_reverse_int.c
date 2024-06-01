/*
    Author: Salma Faragalla
    Description:-Function to reverse an integer
                -Function to find if an integer is a palindrome or not 
                -Function to test the IsPalindrome function 
*/

#include <stdio.h>
int Reverse_Int (int num);
int Is_Palindrome (int num);
void Test_IsPalindrome();

enum state
{
   false , true
};

int main ()
{
    /*int num;
    printf ("Please enter a number : ");
    scanf ("%d",&num);
    printf ("The reversed number is : %d \n",Reverse_Int(num));
    */
    //Is_Palindrome(569965);


    Test_IsPalindrome();
}

int Reverse_Int (int num)
{
    int remainder=0;
    int reversedNumber=0;

    while (num>0)
    {
        remainder=num %10;
        reversedNumber = remainder+ (reversedNumber*10);
        num/=10; 
    }

    return reversedNumber;

}
int Is_Palindrome (int num)
{
    int  reversedNumber=Reverse_Int(num);

    if (reversedNumber==num) 
    {
        printf ("The integer is a palindrome.\n");
        return true;
    }
    
    else 
    {
        printf ("The integer is NOT a palindrome.\n");
        return false;
    }

}

void Test_IsPalindrome()
{
    int  testCases [10] = {0,1,10,33,100,505,789,999,12321,569965};
    int  expected  [10] = {1,1,0, 1, 0  ,1  , 0 ,1  , 1   ,1 }; 

    int n=0;

    for (int i=9 ; i>=0 ; --i )
    {
        if (expected[i]==Is_Palindrome(testCases[i]))
            ++n;
        
    }

    if (n==10)
        printf ("All test cases passed.");
    else 
        printf ("%d/10 test cases passed. ",n);


}
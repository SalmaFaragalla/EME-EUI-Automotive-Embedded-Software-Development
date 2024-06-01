/*
    Author: Salma Faragalla
    Description: Stack implementation using an array
*/

#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    int top;
    int size;
    int* arr;
}stack;

enum state
{
    false,true
};

stack * create_stack (int size);
int Is_Empty (stack *my_stack);
int Is_Full (stack *my_stack);
int push (int value ,  stack *my_stack);
int pop (stack *my_stack , int * result);
void print_stack (stack *my_stack);


int main ()
{
    char continue_stack = 'Y';
    int operationID=0;
    int value;
    int stack_size=0;
    int result;

    printf ("Please enter the stack size : ");
    scanf("%d",&stack_size);
    stack my_stack = *(create_stack(stack_size));
    
    while ('Y'==continue_stack)
    {
        printf ("Please choose from the following operations : \n");
        printf ("1)Push\n");
        printf ("2)Pop\n");
        printf ("3)Print Stack\n");


        printf ("Please enter the operation ID : ");
        scanf ("%d",&operationID);

        while (operationID<1 || operationID>3)
        {
            printf ("Invalid operation ID\n");
            printf ("Please enter the operation ID : ");
            scanf ("%d",&operationID);
        }
        switch (operationID)
        {
            case 1:
                
                printf ("Please enter a value to push : ");
                scanf ("%d",&value);
                push(value,&my_stack);
            break;

            case 2:
                if (pop(&my_stack,&result)==1)
                printf ("The popped value = %d",result);
            break;
        
            case 3:

                print_stack(&my_stack);
            break;
        }

        printf("\n");
        
        printf ("Do you want to continue ? (Y/N) : ");
        scanf (" %c",&continue_stack);
        while (continue_stack != 'Y' && continue_stack != 'N')
        {
            printf("Please enter either Y or N ");
            scanf (" %c",&continue_stack);
        }
        
        printf("\n");
    }

    return 0;

}

stack * create_stack (int stack_size)
{
    stack my_stack;
    stack * my_stack_ptr = &my_stack;
    my_stack.size = stack_size;
    my_stack.arr = malloc(sizeof(int) * stack_size); 
    my_stack.top = -1;

    return my_stack_ptr;

}
int Is_Empty (stack *my_stack)
{
    if (my_stack->top==-1)
    {
        return true;
    }
    return false;
}
int Is_Full (stack *my_stack)
{
    if (my_stack->top== my_stack->size-1)
    {
        return true;
    }
    return false;
}

int push (int value ,  stack *my_stack)
{
    if (Is_Full(my_stack)) 
    {
        printf ("The stack is full !!\n");
        return false;

    }

    ++my_stack->top;
    my_stack->arr[my_stack->top] = value;
    return true;

}

int pop (stack *my_stack , int * result)
{
    if (Is_Empty(my_stack)) 
    {
        printf ("The stack is empty !!\n");
        return false;
    }
    *result=my_stack->arr[my_stack->top];
    --my_stack->top;
    
    return true;
}

void print_stack (stack *my_stack)
{
    
    if (Is_Empty(my_stack))
    {
        printf("The stack is empty !!\n");
        return;
    }

    int n=my_stack->top;
    int result;
    for (int i=0 ; i<=n ; ++i)
    {
        pop(my_stack,&result);
        printf ("%d ",result);
    }
}

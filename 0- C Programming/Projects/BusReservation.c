/*
    Author: Salma Faragalla
    Description: Bus reservation system 
                Exact requirements are attached in a pdf file 
*/

#include <stdio.h>
#include <string.h>

#define BUSSES_NUMBER 3
#define BUS_0_ID 0
#define BUS_1_ID 1
#define BUS_2_ID 2
#define BUS_0_TOTAL_SEATS 12
#define BUS_1_TOTAL_SEATS 20
#define BUS_2_TOTAL_SEATS 30
#define TIME_SIZE 8
#define DATE_SIZE 11
#define POINTS_SIZE 31

#define MAX_RESERVATIONS_NUMBER 3
#define RESERVATION_ID_OFFSET 1000
#define EMPTY -1

#define USERNAME_PASSWORD_SIZE 20

#define MIN_OPERATION_ID 1
#define MAX_OPERATION_ID 3

typedef struct
{
    int id;
    char pickup_time[TIME_SIZE];
    char arrival_time[TIME_SIZE];
    char date[DATE_SIZE];
    char pickup_point[POINTS_SIZE];
    char arrival_point[POINTS_SIZE];
    int available_seats;
    int total_seats;

} bus;

typedef struct
{
    int reservation_id;
    int bus_id;
    int seat_number;

} reservation;

typedef struct
{
    char username[USERNAME_PASSWORD_SIZE];
    char password[USERNAME_PASSWORD_SIZE];
    reservation all_reservations[MAX_RESERVATIONS_NUMBER];
    int number_of_reservations;

} user;

static bus all_busses[BUSSES_NUMBER] =
    {
        {BUS_0_ID, "7:00 AM", "8:30 AM", "12/08/2023", "Giza ", "Sokhna", BUS_0_TOTAL_SEATS, BUS_0_TOTAL_SEATS},
        {BUS_1_ID, "6:00 PM", "9:45 PM", "17/08/2023", "5th district", "Sahel", BUS_1_TOTAL_SEATS, BUS_1_TOTAL_SEATS},
        {BUS_2_ID, "6:00 PM", "9:45 PM", "17/08/2023", "New Capital", "Dahab", BUS_2_TOTAL_SEATS, BUS_2_TOTAL_SEATS}

};

static user current_user = {"salma", "0000", {{EMPTY, EMPTY, EMPTY}, {EMPTY, EMPTY, EMPTY}, {EMPTY, EMPTY, EMPTY}}, 0};

void login(); /*ask for username and password and validate*/
int main_menu();
void book_bus();     /*ask for bus ID , check if seats are available , if yes return true and decrement the number of available seats for that bus, if no return false */
int show_bus_list(); /* displays the array of busses only print */
void cancel_bus();   /*ask user for bus ID , increment number of available seats */

int main()
{
    int operationID = 0;
    char continue_bus = 'Y';
    login();

    while ('Y' == continue_bus)
    {
        operationID = main_menu();
        switch (operationID)
        {
        case 1:
            if (current_user.number_of_reservations == MAX_RESERVATIONS_NUMBER)
            {
                printf("Maximum number of reservations reached.\nPlease cancel a reservation if you want to book a new one \n");
                break;
            }
            if (!show_bus_list())
                break;
            book_bus();
            break;
        case 2:
            cancel_bus();
            break;
        case 3:
            login();
            continue_bus = 'Y';
            continue;
        }

        printf("\nDo you want to go back to main menu ? (Y/N): ");
        scanf(" %c", &continue_bus);
        while (continue_bus != 'Y' && continue_bus != 'N')
        {
            printf("Please enter either Y or N:");
            scanf(" %c", &continue_bus);
        }

        printf("\n");
    }
}

void login()
{
    char username[USERNAME_PASSWORD_SIZE];
    char password[USERNAME_PASSWORD_SIZE];
    printf("---------------------------------------------------\n");
    printf("                      Login                         \n");
    printf("---------------------------------------------------\n");

    fflush(stdin);
    printf("Please Enter your username : ");
    gets(username);
    while (strcmp(username, current_user.username))
    {
        printf("Username is incorrect.\nPlease enter a correct username : ");
        gets(username);
    }

    printf("Please Enter your password : ");
    gets(password);

    while (strcmp(password, current_user.password))
    {
        printf("Password is incorrect.\nPlease enter the correct password : ");
        gets(password);
    }
}

int main_menu()
{
    int operation = 0;
    printf("---------------------------------------------------\n");
    printf("                     Main Menu                     \n");
    printf("---------------------------------------------------\n");
    printf("1-Show bus list & book \n2-Cancel\n3-Logout\n");
    printf("\nPlease enter the ID of the option you would like to select: ");
    scanf("%d", &operation);

    while (operation < MIN_OPERATION_ID || operation > MAX_OPERATION_ID)
    {
        printf("Invalid ID.\n Please choose an option ID from the above list: ");
        scanf("%d", &operation);
    }
    return operation;
}

int show_bus_list()
{
    char continue_book;
    printf("---------------------------------------------------\n");
    printf("                    Available busses               \n");

    for (int i = 0; i < BUSSES_NUMBER; ++i)
    {

        printf("---------------------------------------------------\n");
        printf("Bus ID: %d\n\nDate: %s\n\nPickup time: %s \nPickup point: %s\n\n", all_busses[i].id, all_busses[i].date, all_busses[i].pickup_time, all_busses[i].pickup_point);
        printf("Arrival Time: %s \nArrival point: %s\n\nTotal seats: %d\nAvailable seats: %d\n", all_busses[i].arrival_time, all_busses[i].arrival_point, all_busses[i].total_seats, all_busses[i].available_seats);
    }
    printf("---------------------------------------------------\n");

    printf("Do you want to book a bus? (Y/N): ");
    scanf(" %c", &continue_book);
    while (continue_book != 'Y' && continue_book != 'N')
    {
        printf("Please enter either Y or N:");
        scanf(" %c", &continue_book);
    }

    if (continue_book == 'Y')
        return 1;
    else
        return 0;
}

void book_bus()
{
    int busID = 0;
    int seats_num = 0;
    int reservation_index;

    printf("---------------------------------------------------\n");
    printf("                     Booking                       \n");
    printf("---------------------------------------------------\n");

    printf("Please enter the ID of the bus you want to book: ");
    scanf("%d", &busID);

    while (busID < 0 || busID > BUSSES_NUMBER)
    {
        printf("Invalid ID.\nPlease enter the ID of the bus you want to book: : ");
        scanf("%d", &busID);
    }

    while (all_busses[busID].available_seats == 0)
    {
        printf("This bus is fully booked.\nPlease enter another bus id:");
        scanf("%d", &busID);
    }

    printf("Please enter the number of seats you want to book: ");
    scanf("%d", &seats_num);

    while (seats_num <= 0)
    {
        printf("The number of seats can't be negative or zero\n");
        printf("Please enter a positive number of seats %d: ", all_busses[busID].available_seats);
        scanf("%d", &seats_num);
    }

    while (seats_num > all_busses[busID].available_seats)
    {
        printf("The number of seats requested is greater than the number of available seats\n");
        printf("Please enter a number of seats less than or equal to %d: ", all_busses[busID].available_seats);
        scanf("%d", &seats_num);
    }

    all_busses[busID].available_seats -= seats_num;

    for (reservation_index = 0; reservation_index < MAX_RESERVATIONS_NUMBER; ++reservation_index)
    {
        if (current_user.all_reservations[reservation_index].reservation_id == EMPTY)
            break;
    }

    current_user.all_reservations[reservation_index].bus_id = busID;
    current_user.all_reservations[reservation_index].seat_number = seats_num;

    switch (reservation_index)
    {
    case 0:
        current_user.all_reservations[reservation_index].reservation_id = RESERVATION_ID_OFFSET + 1;
        break;
    case 1:
        current_user.all_reservations[reservation_index].reservation_id = RESERVATION_ID_OFFSET + 2;
        break;
    case 2:
        current_user.all_reservations[reservation_index].reservation_id = RESERVATION_ID_OFFSET + 3;
        break;
    }
    current_user.number_of_reservations++;

    printf("---------------------------------------------------\n");
    printf("Thank you for booking your trip with us !! \nYour reservation ID is: %d\n", current_user.all_reservations[reservation_index].reservation_id);

    return;
}
void show_reserved_busses()
{
    printf("---------------------------------------------------\n");
    printf("                   Reservations                    \n");
    for (int i = 0; i < MAX_RESERVATIONS_NUMBER; ++i)
    {
        if (current_user.all_reservations[i].reservation_id != EMPTY)
        {
            printf("---------------------------------------------------\n");
            printf("Reservation ID: %d\n", current_user.all_reservations[i].reservation_id);
            printf("For Bus with ID: %d & Number of seats:%d \n", current_user.all_reservations[i].bus_id, current_user.all_reservations[i].seat_number);
        }
    }

    printf("---------------------------------------------------\n");
}
void cancel_bus()
{
    char found_reserv = 0;
    int reservation_index = 0;
    int resev_ID;

    if (current_user.number_of_reservations == 0)
    {
        printf("\nYou haven't made any reservations.\n");
        return;
    }

    show_reserved_busses();
    printf("Please enter the reservation ID you want to cancel: ");
    scanf("%d", &resev_ID);

    for (reservation_index = 0; reservation_index < MAX_RESERVATIONS_NUMBER; ++reservation_index)
    {
        if (current_user.all_reservations[reservation_index].reservation_id != EMPTY)
        {
            if (current_user.all_reservations[reservation_index].reservation_id == resev_ID)
            {
                found_reserv = 1;
                break;
            }
        }
    }

    while (!found_reserv)
    {
        printf("Invalid ID \nPlease choose a reservation ID from the above list:");
        scanf("%d", &resev_ID);
        for (reservation_index = 0; reservation_index < MAX_RESERVATIONS_NUMBER; ++reservation_index)
        {
            if (current_user.all_reservations[reservation_index].reservation_id != EMPTY)
            {
                if (current_user.all_reservations[reservation_index].reservation_id == resev_ID)
                {
                    found_reserv = 1;
                    break;
                }
            }
        }
    }

    current_user.all_reservations[reservation_index].reservation_id = EMPTY;
    all_busses[current_user.all_reservations[reservation_index].bus_id].available_seats += current_user.all_reservations[reservation_index].seat_number;
    current_user.number_of_reservations--;

    printf("Your trip has been cancelled successfully.\n");

    return;
}

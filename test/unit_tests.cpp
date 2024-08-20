#include "SenseHat.hpp"
#include <string>

int main(int arg, char **argv)
{
    Sensehat my_sensehat;
    int8_t status = my_sensehat.init(1);
    if (argv[1] == std::string("1"))
    {
        switch (status)
        {
        case 31:
            printf("Sensehat fully functional!\n");
            exit(0);
            break;
        case -1:
            printf("Failure to open i2c bus\n");
            perror("oups!");
            exit(1);
            break;
        default:
            printf("At least one i2c device fails!\n");
            printf("Status: %d", status);
            exit(1);
            break;
        }
    }
}

// 编译 g++ -o send_message publish.cpp -llcm

#include <lcm/lcm-cpp.hpp>

#include "armor_msg.hpp"

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    armor_msg my_data;
    my_data.position[0] = 1;
    my_data.position[1] = 3;
    my_data.position[2] = 2;
    while(1)
    {
        lcm.publish("armor_msg", &my_data);
    }
    return 0;
}

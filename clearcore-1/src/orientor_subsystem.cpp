 /**
 * @file orientor_subsystem.cpp
 * @copyright Copyright (c) 2023 Vebu Labs. All rights reserved.
 * @brief This library is used to control the orientor subsystem
 * @author Mike Lui
*/

#include "orientor_subsystem.hpp"

void OrientorFSMClass::setup()
{
    if(!has_setup){
        has_setup = true;
    }
}

void OrientorFSMClass::read_interfaces()
{

}

void OrientorFSMClass::run()
{

}

void OrientorFSMClass::write_interfaces()
{

}

OrientorFSMClass orientor;
#ifndef _BLADERF_COMMON_H_
#define _BLADERF_COMMON_H_

#include <cstdint>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>


// ----------------------------------------------------------------------------
inline std::ostream& operator<< (
    std::ostream& out,
    const struct bladerf_devinfo &item
    )
{
    out << "BladeRF Device Information: " << std::endl;
    out << "  backend:       " << std::string(bladerf_backend_str(item.backend)) << std::endl;
    out << "  serial number: " << std::string(item.serial) << std::endl;
    out << "  usb_bus:       " << (uint32_t)item.usb_bus << std::endl;
    out << "  usb_addr:      " << (uint32_t)item.usb_addr << std::endl;
    out << "  instance:      " << item.instance << std::endl;
    out << "  manufacturer:  " << std::string(item.manufacturer) << std::endl;
    out << "  product:       " << std::string(item.product) << std::endl;
    out << std::endl;
    return out;
}


//-----------------------------------------------------------------------------
int select_bladerf(int num_devices, struct bladerf_devinfo* device_list)
{
    uint32_t idx;
    std::string console_input;

    if (num_devices > 0)
    {

        for (idx = 0; idx < num_devices; ++idx)
        {
            std::cout << "BladeRF Device [" << idx << "]: " << std::string(device_list[idx].serial) << std::endl;
            //std::cout << device_list[idx] << std::endl;
        }
        
        std::cout << "Select BladeRF device number: ";
        std::getline(std::cin, console_input);
        return std::stoi(console_input);
    }
    else
    {
        std::cout << "Could not detect any bladeRF devices.  Check connections and try again..." << std::endl;
    }

    return -1;

}   // end of get_device_list

// ----------------------------------------------------------------------------
void bladerf_status(int status)
{
    if (status != 0)
    {
        std::cout << "Unable to open device: " << std::string(bladerf_strerror(status)) << std::endl;
        exit(status);
    }
    //return status;

}

#endif  // _BLADERF_COMMON_H_

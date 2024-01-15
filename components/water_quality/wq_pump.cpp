#include "wq_pump.h"

namespace esphome {
namespace water_quality {

void Pump::Pump_driver(float pwm[])
{     
    auto start = std::chrono::high_resolution_clock::now();
               
    // uint8_t pump[6] = get_Pump_Status();

    // std::thread thread1(Dosing_Controller, pump);
    // Circulation_Controller(pump);
    
    // thread1.join();

              
    // std::cout << "Pump_driver\n";
    
    // uint16_t* dose = get_Pump_Dose();
    // uint16_t* circ = get_Pump_Circulation();
    // uint8_t* type = get_Pump_Type();
    // uint8_t* mode = get_Pump_Mode();
    uint16_t (*tot)[6][2] = get_Pump_Total();

    for (size_t i = 0; i < 6; i++)
    {
        while ((dose[i] > 0 && type[i] == 1) || (circ[i] > 0 && type[i] == 2))
            if (mode[i] == 1)
            {
            // Dosing_Controller(pwm);
            std::thread thread1(&Pump::Dosing_Controller, this, pwm);
            // Circulation_Controller(pwm);
            std::thread thread2(&Pump::Circulation_Controller, this, pwm);
            
            thread1.join();
            thread2.join();
            }
        
    
    // std::thread thread1(&Pump::Dosing_Controller, this, pwm, i);
    // thread1.join();
        // std::this_thread::sleep_for(std::chrono::milliseconds (static_cast<uint16_t>(pwm[i] * 1000)));
   
    //     std::cout << "pwm[" << i << "] = " << pwm[i] << "\n";

    
        // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";
    
    ESP_LOGD(TAG,"Pump_Total[%d] = %d.%03d", i, (*tot)[i][0], (*tot)[i][1]);
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // std::cout << "Geçen süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";

    ESP_LOGI(TAG,"Geçen süre: %f saniye", duration.count() / 1000);
    
}
void Pump::Dosing_Controller(float pump[])
{
    // auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "Dosing_Controller\n";
    uint8_t* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    uint16_t* dose = get_Pump_Dose();
    uint16_t (*tot)[6][2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float mint, min[6];

    std::copy(pump, pump + 6, min);
    std::sort(min, min + 6);

    for (size_t i = 0; i < 6; ++i) 
    {
        if (min[i] > 0)
        {
            mint = min[i];
            break;
        }
        else
            mint = 0;
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 1)
        {

            // std::cout << "reset" << i << ": " << reset[i] << "\n";
            // std::cout << "mode" << i << ": " << (mode[i] ? "true" : "false") << "\n";
            // std::cout << "dose" << i << ": " << dose[i] << "\n";
            
            if (stat[i] == 1)
            {
                //  std::cout << "Pump_stat[" << i << "] = " << (stat[i] ? "true" : "false") << "\n";
                (*tot)[i][0] += static_cast<uint16_t>(((*tot)[i][1] + (dose[i] > 0 ? calib[i] : 0) * mint)) / 1000;
                (*tot)[i][1] = static_cast<uint16_t>(((*tot)[i][1] + (dose[i] > 0 ? calib[i] : 0) * mint)) % 1000;
                
                dose[i] -= (pump[i] > mint ? mint : pump[i]) * calib[i];
            }

            if (reset[i])
            {
                (*tot)[i][0] = 0;
                (*tot)[i][1] = 0;
            }

            if (mode[i])
                pump[i] = dose[i] > calib[i] ? 1 : static_cast<float>(dose[i]) / calib[i];
            else if (!mode[i])
                pump[i] = 0;  
            
            if (pump[i] > 0)
                stat[i] = 1;
               
        // std::cout << "pump" << i << " " << pump[i] << "\n";
            // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";
        
        // std::this_thread::sleep_for(std::chrono::milliseconds (dose[i] > calib[i] ? 1000 : dose[i] * 1000 / calib[i]));
        
        }
            
        // std::cout << i << "    " << pump[i] << "    " << (mode[i] ? "true" : "false") << "    " << dose[i] << "\n";
            
             
        // std::cout << pump[i] << " " << dose[i] << "\n";
    }

    // std::cout << "mint " << mint << "\n";
 
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint16_t>(mint * 1000)));

    set_Pump_Status(stat);
         
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
     
    // std::cout << "Süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";

}
void Pump::Circulation_Controller(float pump[])
{
    // auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "Dosing_Controller\n";
    uint8_t* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    uint16_t* circ = get_Pump_Circulation();
    uint16_t (*tot)[6][2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float mint, min[6];

    std::copy(pump, pump + 6, min);
    std::sort(min, min + 6);

    for (size_t i = 0; i < 6; ++i) 
    {
        if (min[i] > 0)
        {
            mint = min[i];
            break;
        }
        else
            mint = 0;
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 2)
        {

            // std::cout << "reset" << i << ": " << reset[i] << "\n";
            // std::cout << "mode" << i << ": " << (mode[i] ? "true" : "false") << "\n";
            // std::cout << "circ" << i << ": " << circ[i] << "\n";
            
            if (stat[i] == 1)
            {
                //  std::cout << "Pump_stat[" << i << "] = " << (stat[i] ? "true" : "false") << "\n";
                (*tot)[i][0] += static_cast<uint16_t>(((*tot)[i][1] + (circ[i] > 0 ? calib[i] : 0) * mint)) / 1000;
                (*tot)[i][1] = static_cast<uint16_t>(((*tot)[i][1] + (circ[i] > 0 ? calib[i] : 0) * mint)) % 1000;
                
                circ[i] -= (pump[i] > mint ? mint : pump[i]) * calib[i];
            }

            if (reset[i])
            {
                (*tot)[i][0] = 0;
                (*tot)[i][1] = 0;
            }

            if (mode[i])
                pump[i] = circ[i] > calib[i] ? 1 : static_cast<float>(circ[i]) / calib[i];
            else if (!mode[i])
                pump[i] = 0;  
            
            if (pump[i] > 0)
                stat[i] = 1;
               
        // std::cout << "pump" << i << " " << pump[i] << "\n";
            // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";
        
        // std::this_thread::sleep_for(std::chrono::milliseconds (circ[i] > calib[i] ? 1000 : circ[i] * 1000 / calib[i]));
        
        }
            
        // std::cout << i << "    " << pump[i] << "    " << (mode[i] ? "true" : "false") << "    " << circ[i] << "\n";
            
             
        // std::cout << pump[i] << " " << circ[i] << "\n";
    }

    // std::cout << "mint " << mint << "\n";
 
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint16_t>(mint * 1000)));

    set_Pump_Status(stat);
         
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
     
    // std::cout << "Süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";
    // set_Pump_Total(*tot);
}
// void Pump::pump_total()
// {
//     for (size_t i = 0; i < Pump_Type.size(); i++)
//     {
//         if (Pump_Type[i] == 1)
//         {
//             if (Pump_Dose[i] >= 1)
//             {
//                 Pump_Total[i][1] += Pump_Dose[i]%2;

//                 if (Pump_Dose[i] == 1)
//                     Pump_Dose[i] = 0;
//                 else
//                 {
//                     if (Pump_Mode[i] == 2)
//                     {
//                         Pump_Status[i] = 3;
//                         break;
//                     }
//                     Pump_Dose[i] /= 2;
//                     Pump_Total[i][0] += (Pump_Total[i][0] + Pump_Dose[i]) / 1000;
//                     Pump_Total[i][1] = (Pump_Total[i][1] + Pump_Dose[i]) % 1000;
//                 }
//             }
//             Pump_Mode[i] = 0;
//             Pump_Status[i] = 2;
//         }
//         if (Pump_Type[i] == 2)
//         {
//             if (Pump_Circulation[i] >= 1)
//             {
//                 ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, Pump_Circulation[i]);
//                 Pump_Total[i][1] += Pump_Circulation[i]%2;
//                 if (Pump_Circulation[i] == 1)
//                     Pump_Circulation[i] = 0;
//                 else
//                 {
//                     if (Pump_Mode[i] == 2)
//                     {
//                         Pump_Status[i] = 3;
//                         Pump_Circulation[i] = 0;
//                     }
//                     Pump_Circulation[i] /= 2;
//                     Pump_Total[i][0] += (Pump_Total[i][0] + Pump_Circulation[i]) / 1000;
//                     Pump_Total[i][1] = (Pump_Total[i][1] + Pump_Circulation[i]) % 1000;
//                 }
//             }
//             Pump_Mode[i] = 0;
//             Pump_Status[i] = 2;
//         }
//         ESP_LOGD(TAG,"Pump_Total[%d] = %d.%03d", i, Pump_Total[i][0], Pump_Total[i][1]);

//     }
// }

}  // namespace water_quality
}  // namespace esphome
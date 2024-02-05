#include "wq_pump.h"

namespace esphome {
namespace water_quality {

void Pump::Timer_Setup()
{
    // Timer'ı başlat
    esp_timer_create_args_t timer_args = {
        .callback = &Pump::Timer0,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = nullptr,
    };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);

    // Timer'ı başlat ve her 1 saniyede bir çağrılmasını sağla
    esp_timer_start_periodic(timer, 1 * 1000000); // 1 saniye (mikrosaniye cinsinden)
}

	static uint32_t multFactor = 0;
	static uint32_t timers = 0;
void IRAM_ATTR Pump::Timer0(void* arg)
{
    timers = millis();
    ESP_LOGI(TAG, "timer = %d", timers - multFactor);
    multFactor = timers;
}

void Pump::Pump_driver(float pwm[])
{     
    // auto start = std::chrono::high_resolution_clock::now();
               
    // float pump[6];

    // std::thread thread1(Dosing_Controller, pump);
    // Circulation_Controller(pump);
    
    // thread1.join();

              
    // std::cout << "Pump_driver\n";
    
    uint8_t* stat = get_Pump_Status();
    uint16_t (*tot)[2] = get_Pump_Total();

    // for (size_t i = 0; i < 6; i++)
    // {
    //     pump[i] = pwm[i];
    // }
    //     do
    //     {
    //     while ((dose[i] > 0 && type[i] == 1) || (circ[i] > 0 && type[i] == 2))
    //         if (mode[i] == 1)
    //         {
    uint8_t stat_[6];
    for (size_t i = 0; i < 6; i++)
        stat_[i] = stat[i];

            Dosing_Controller(stat);
            Circulation_Controller(stat);
            // std::thread thread1(&Pump::Dosing_Controller, this, pwm);
            // std::thread thread2(&Pump::Circulation_Controller, this, pwm);
            
            // thread1.join();
            // thread2.join();
            
        // ESP_LOGI(TAG, "pwm[%d] = %f", i, pwm[i]);
        // } 
        // while (pwm[i] > 0);

    for (size_t i = 0; i < 6; i++)
    {
        if (stat[i] == 1)
            pwm[i] = 1;
        else
        {
            if (stat[i] != stat_[i])
                ESP_LOGD(TAG, "Pump_Total[%d] = %d.%03d", i, tot[i][0], tot[i][1]);

            pwm[i] = 0;
        }
    }
        
    
    // std::thread thread1(&Pump::Dosing_Controller, this, pwm, i);
    // thread1.join();
        // std::this_thread::sleep_for(std::chrono::milliseconds (static_cast<uint16_t>(pwm[i] * 1000)));
   
    //     std::cout << "pwm[" << i << "] = " << pwm[i] << "\n";

    // }

    // for (size_t i = 0; i < 6; i++)
    // {
        // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";

    // }

    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // std::cout << "Geçen süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";

    // ESP_LOGI(TAG, "Geçen süre: %f saniye", duration.count() / 1000);
    
}
void Pump::Dosing_Controller(uint8_t stat[])
{
    // auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "Dosing_Controller\n";
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    float* dose = get_Pump_Dose();
    uint16_t (*tot)[2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float mint, min[6];

    // std::copy(pump, pump + 6, min);
    std::copy(stat, stat + 6, min);
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
                tot[i][0] += static_cast<uint16_t>(tot[i][1] + (dose[i] > 0 ? calib[i] : 0) * mint * 10) / 10000;
                tot[i][1] = static_cast<uint16_t>(tot[i][1] + (dose[i] > 0 ? calib[i] : 0) * mint * 10) % 10000;
                
                dose[i] -= (stat[i] > mint ? mint : stat[i]) * calib[i];
            }

            // if (stat[i] == 1 && !(dose[i] > 0))
            // {
            //     ESP_LOGD(TAG, "Pump_Total[%d] = %d.%03d", i, (*tot)[i][0], (*tot)[i][1]);
            // }

            if (reset[i])
            {
                tot[i][0] = 0;
                tot[i][1] = 0;
            }

            switch (mode[i])
            {
            case 0:
                // pump[i] = 0; 
                stat[i] = 0;
                break;
            case 1:
                if (dose[i] > 0)
                    if (i % 2 == 0 || (i % 2 == 1 && mode[i - 1] == 0))
                    {
                        stat[i] = dose[i] > calib[i] ? 1 : static_cast<float>(dose[i]) / calib[i];
                        // stat[i] = 1;
                    }
                    else
                    {
                        // pump[i] = 0; 
                        stat[i] = 0;
                    }
                else
                {
                    // pump[i] = 0; 
                    stat[i] = 2;
                    mode[i] = 0;
                }
                break;
            case 2:
                // pump[i] = 0; 
                stat[i] = 3;
                break;
            
            default:
                break;
            }


               
        // std::cout << "pump" << i << " " << pump[i] << "\n";
            // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";
        
        // std::this_thread::sleep_for(std::chrono::milliseconds (dose[i] > calib[i] ? 1000 : dose[i] * 1000 / calib[i]));
        
        }
            
        // std::cout << i << "    " << pump[i] << "    " << (mode[i] ? "true" : "false") << "    " << dose[i] << "\n";
            
             
        // std::cout << pump[i] << " " << dose[i] << "\n";
    }

    // std::cout << "mint " << mint << "\n";
 
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint16_t>(mint * 1000)));
         
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
     
    // std::cout << "Süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";

}
void Pump::Circulation_Controller(uint8_t stat[])
{
    // auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "Dosing_Controller\n";
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    float* circ = get_Pump_Circulation();
    uint16_t (*tot)[2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float mint, min[6];

    std::copy(stat, stat + 6, min);
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
                tot[i][0] += static_cast<uint16_t>(tot[i][1] + (circ[i] > 0 ? calib[i] : 0) * mint * 10) / 10000;
                tot[i][1] = static_cast<uint16_t>(tot[i][1] + (circ[i] > 0 ? calib[i] : 0) * mint * 10) % 10000;
                
                circ[i] -= (stat[i] > mint ? mint : stat[i]) * calib[i];   
            }

            // if (stat[i] == 1 && !(circ[i] > 0))
            // {
            //     ESP_LOGD(TAG, "Pump_Total[%d] = %d.%03d", i, (*tot)[i][0], (*tot)[i][1]);
            // }

            if (reset[i])
            {
                tot[i][0] = 0;
                tot[i][1] = 0;
            }

            switch (mode[i])
            {
            case 0:
                // pump[i] = 0; 
                stat[i] = 0;
                break;
            case 1:
                if (circ[i] > 0)
                    if (i % 2 == 0 || (i % 2 == 1 && mode[i - 1] == 0))
                    {
                        stat[i] = circ[i] > calib[i] ? 1 : static_cast<float>(circ[i]) / calib[i];
                        // stat[i] = 1;
                    }
                    else
                    {
                        // pump[i] = 0; 
                        stat[i] = 0;
                    }
                else
                {
                    // pump[i] = 0; 
                    stat[i] = 2;
                    mode[i] = 0;
                }
                break;
            case 2:
                // pump[i] = 0; 
                stat[i] = 3;
                break;
            
            default:
                break;
            }
               
        // std::cout << "pump" << i << " " << pump[i] << "\n";
            // std::cout << "Pump_Total[" << i << "] = " << (*tot)[i][0] << "." <<  ((*tot)[i][1] < 100 ? "0" + std::to_string((*tot)[i][1]) :  std::to_string((*tot)[i][1])) << "\n";
        
        // std::this_thread::sleep_for(std::chrono::milliseconds (circ[i] > calib[i] ? 1000 : circ[i] * 1000 / calib[i]));
        
        }
            
        // std::cout << i << "    " << pump[i] << "    " << (mode[i] ? "true" : "false") << "    " << circ[i] << "\n";
            
             
        // std::cout << pump[i] << " " << circ[i] << "\n";
    }

    // std::cout << "mint " << mint << "\n";
 
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint16_t>(mint * 1000)));
         
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
     
    // std::cout << "Süre: " << static_cast<float>(duration.count()) / 1000 << " saniye\n";
    // set_Pump_Total(*tot);
}

}  // namespace water_quality
}  // namespace esphome
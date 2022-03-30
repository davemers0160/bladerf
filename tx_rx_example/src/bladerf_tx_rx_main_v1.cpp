// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

#elif defined(__linux__)

#endif
// ArrayFire Includes
#include <arrayfire.h>

#include <cstdint>
#include <iostream>
#include <sstream>
#include <deque>
#include <atomic>
#include <thread>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"

// Project Includes
#include <bladerf_common.h>

// ----------------------------------------------------------------------------
// Globals
std::atomic<bool> rx_run(false);
std::atomic<bool> rx_complete(false);

uint32_t timeout_ms = 10000;

template<typename T>
void save_complex_data(std::string filename, std::vector<std::complex<T>> data)
{
    std::ofstream data_file;

    T r, q;

    data_file.open(filename, ios::out | ios::binary);

    if (!data_file.is_open())
    {
        std::cout << "Could not save data. Closing... " << std::endl;
        //std::cin.ignore();
        return;
    }

    data_file.write(reinterpret_cast<const char*>(data.data()), 2 * data.size() * sizeof(T));

    //for (uint64_t idx = 0; idx < data.size(); ++idx)
    //{
    //    //r = data[idx].real();
    //    //q = data[idx].imag();
    //    data_file.write(reinterpret_cast<const char*>(data[idx].real()), sizeof(T));
    //    data_file.write(reinterpret_cast<const char*>(data[idx].imag()), sizeof(T));
    //}
    data_file.close();
}

// ----------------------------------------------------------------------------
template<typename T>
std::vector<T> maximal_length_sequence(uint16_t N, uint16_t rep, T amplitude)
{
    uint64_t idx, jdx;
    uint16_t tmp;
    std::vector<T> sr;

    std::vector<uint16_t> taps = { 0, (uint16_t)(N - 1) };

    // initialize the register
    std::deque<uint8_t> r(N, 0);
    r[0] = 1;

    // shift register 
    uint64_t sr_size = (1 << N) - 1;

    for (idx = 0; idx < sr_size; ++idx)
    {
        sr.insert(sr.end(), rep, amplitude * (2*r[N-1] - 1));

        tmp = 0;
        for (jdx = 0; jdx < taps.size(); ++jdx)
        {
            tmp += r[taps[jdx]];
        }
        tmp = tmp % 2;

        r.push_front(tmp);
        r.pop_back();
    }

    return sr;
}   // end of maximal_length_sequence

// ----------------------------------------------------------------------------
void RX(struct bladerf* dev, std::vector<int16_t> &samples)
{
    int blade_status;
    uint32_t num_samples = samples.size() >> 1;

    while (rx_run)
    {
        if (rx_complete == false)
        {
            blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                rx_run = false;
            }

            rx_complete = true;
        }
        // try to compute the fft using the arrayfire library
        //std::vector<std::complex<float>> c_samples(num_samples);
        //int index = 0;
        //for (idx = 0; idx < samples.size(); idx += 2)
        //{
        //    c_samples[index++] = std::complex<float>((float)samples[idx], (float)samples[idx + 1]);
        //}
    }
}

// ----------------------------------------------------------------------------
void TX(struct bladerf* dev, std::vector<std::complex<int16_t>>& samples)
{
    uint32_t num_samples = samples.size();

    int blade_status = bladerf_sync_tx(dev, (int16_t*)samples.data(), num_samples, NULL, timeout_ms);

    if (blade_status != 0)
    {
        std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
        //return blade_status;
    }
}

// ----------------------------------------------------------------------------



// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    // bladeRF variable
    struct bladerf_devinfo *device_list = NULL;
    struct bladerf_devinfo dev_info;
    struct bladerf* dev;
    int bladerf_num;
    int blade_status;
    bladerf_channel rx = BLADERF_CHANNEL_RX(0);
    bladerf_channel tx = BLADERF_CHANNEL_TX(0);
    bladerf_frequency freq = 137000000; //162425000;
    bladerf_sample_rate fs = 624000;
    bladerf_bandwidth bw = 624000;
    bladerf_gain rx1_gain = 30;
    bladerf_gain tx1_gain = 10;

    int64_t span = 500000;

    uint32_t num_rx_samples = 1000000;
    std::vector<int16_t> rx_samples(num_rx_samples*2);
    //uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 2 * 8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    double t;
    int16_t amplitude = 1600;

    std::string chirp_filename = "../recordings/test_chirp.bin";
    std::string data_filename = "../recordings/test_record_chirp.bin";
    std::ofstream data_file, chirp_file;

    if (argc == 2)
    {
        std::string param_filename = argv[1];

        read_bladerf_params(param_filename, freq, fs, bw, rx1_gain);
    }

#ifdef USE_ARRAYFIRE
    af::setBackend(AF_BACKEND_CPU);
    af::info();

    std::cout << std::endl << std::endl;

    // array fire variables
    af::array raw_data, fft_data;

#endif // USE_ARRAYFIRE

    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
        std::cin.ignore();
        return 0;
    }

    std::cout << std::endl;

    try{
      
        blade_status = bladerf_open(&dev, ("*:serial=" +  std::string(device_list[bladerf_num].serial)).c_str());
        if (blade_status != 0)
        {
            std::cout << "Unable to open device: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }

        blade_status = bladerf_get_devinfo(dev, &dev_info);
        if (blade_status != 0)
        {
            std::cout << "Unable to get the device info: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            return blade_status;
        }
        std::cout << std::endl << dev_info << std::endl;

        // set the frequency, sample_rate and bandwidth
        blade_status = bladerf_set_frequency(dev, rx, freq);
        blade_status = bladerf_get_frequency(dev, rx, &freq);
        blade_status = bladerf_set_sample_rate(dev, rx, fs, &fs);
        blade_status = bladerf_set_bandwidth(dev, rx, bw, &bw);

        blade_status = bladerf_set_frequency(dev, tx, freq);
        blade_status = bladerf_set_sample_rate(dev, tx, fs, &fs);
        blade_status = bladerf_set_bandwidth(dev, tx, bw, &bw);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);
        blade_status = bladerf_sync_config(dev, BLADERF_TX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

        // enable the rx & tx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);
        blade_status = bladerf_enable_module(dev, BLADERF_TX, true);

        // the gain must be set after the module has been enabled
        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, rx, rx1_gain);
        blade_status = bladerf_get_gain(dev, rx, &rx1_gain);
        blade_status = bladerf_set_gain_mode(dev, tx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, tx, tx1_gain);
        blade_status = bladerf_get_gain(dev, tx, &tx1_gain);

        // receive the samples 
        // the *2 is because one sample consists of one I and one Q.  The data should be packed IQIQIQIQIQIQ...
        //samples.resize(num_samples*2);

        double freq_step = (fs)/(double)num_rx_samples;

        double f_min = (freq - (span>>1)) * 1.0e-6;
        double f_max = (freq + (span>>1)) * 1.0e-6;

        uint32_t sp = (uint32_t)((fs - span) / (2.0 * freq_step));
        uint32_t sp2 = (uint32_t)(span / freq_step);

        double scale = 1.0 / (double)(num_rx_samples);

        // ----------------------------------------------------------------------------
        // create the transmit samples
        // c = 299792458 m/s ==> 1/c = 3.3356409519815204957557671447492e-9 s/m
        // for an object 1m away travel time is 0.00000000333564 s
        // for 10 MSps => 0.0333564 samples
        // for 20 MSps => 0.0667128 samples
        // for 40 MSps => 0.1334256 samples
        std::vector<int16_t> pulse = maximal_length_sequence<int16_t>(4, 4, amplitude);

        pulse.insert(pulse.end(), 2048 - pulse.size(), 0);


        std::vector<std::complex<int16_t>> tx_c(pulse.size());
        int index = 0;
        for (idx = 0; idx < pulse.size(); ++idx)
        {
            tx_c[idx] = std::complex<int16_t>(pulse[idx], 0);
        }

        save_complex_data(chirp_filename, tx_c);

        rx_run = true;
        rx_complete = true;



        // start the rx thread
        std::thread rx_thread(RX, dev, std::ref(rx_samples));

        rx_complete = false;
        while (rx_complete == false);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:       " << fs << std::endl;
        std::cout << "freq:  " << freq << std::endl;
        std::cout << "bw:    " << bw << std::endl;
        std::cout << "rx1_gain: " << rx1_gain << std::endl;
        std::cout << "tx1_gain: " << tx1_gain << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;
/*
#ifdef USE_ARRAYFIRE

        af::Window myWindow(800, 800, "FFT example: ArrayFire");
        //af::array X = af::seq(0, num_samples - 1, 1);
        af::array X = af::seq(sp+1, (sp+sp2+1), 1);
        //auto x2d = X.dims(0);

        //af::array f = af::seq(f_min, f_max - (freq_step * 1.0e-6), (freq_step * 1.0e-6));
        af::array f = af::seq(f_min, f_max, (freq_step * 1.0e-6));

        myWindow.setAxesLimits(f_min, f_max - (freq_step * 1.0e-6), -120, -20, true);
        myWindow.setAxesTitles("Frequency (MHz)", "Power (dBm)");

        while (!myWindow.close())

#else

        while(1)
#endif

        {
            */
            rx_complete = false;

            for(uint32_t jdx=0; jdx<8; ++jdx)
                TX(dev, tx_c);
            //TX(dev, tx_c);
            //TX(dev, tx_c);
            //TX(dev, tx_c);
            //TX(dev, tx_c);
            //TX(dev, tx_c);
            //TX(dev, tx_c);

            while (rx_complete == false)
                std::cout << ".";

            std::cout << std::endl;


            //blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
            //if (blade_status != 0)
            //{
            //    std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
            //    return blade_status;
            //}

            //// try to compute the fft using the arrayfire library
            //std::vector<std::complex<float>> c_samples(num_samples);
            //int index = 0;
            //for (idx = 0; idx < samples.size(); idx += 2)
            //{
            //    c_samples[index++] = std::complex<float>((float)samples[idx], (float)samples[idx + 1]);
            //}

#ifdef USE_ARRAYFIRE
            //raw_data = af::array(num_samples, (af::cfloat*)c_samples.data())*(1/2048.0);

            //af::fftInPlace(raw_data, scale);
            ////auto a1 = af::abs(raw_data).host<float>();

            //fft_data = 20 * af::log10(af::shift(af::abs(raw_data), (num_samples >> 1)));

            //// show the results of the FFT in the window
            //myWindow.plot(f, fft_data(X));
            //myWindow.show();
#endif
            data_file.open(data_filename, ios::out | ios::binary);

            if (!data_file.is_open())
            {
                std::cout << "Could not save data. Closing... " << std::endl;
                std::cin.ignore();
                return 0;
            }

            data_file.write(reinterpret_cast<const char*>(rx_samples.data()), rx_samples.size() * sizeof(int16_t));
            data_file.close();

            std::cout << "Data save complete!" << std::endl;

            std::cout << "Press enter to continue...";
            std::cin.ignore();
        //}

        rx_run = false;
        rx_thread.join();

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);
        blade_status = bladerf_enable_module(dev, BLADERF_TX, false);

        bladerf_close(dev);
    }

#ifdef USE_ARRAYFIRE
    catch (af::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
        std::cin.ignore();
    }
#else
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }
#endif

    
    return 0;
    
}   // end of main

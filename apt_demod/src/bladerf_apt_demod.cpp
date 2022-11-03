// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

#elif defined(__linux__)

#endif

// ArrayFire Includes
// #ifdef USE_ARRAYFIRE
// #include <arrayfire.h>
// #endif

#include <cstdint>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>

#include <complex>

// bladeRF includes
//#include <libbladeRF.h>
//#include <bladeRF2.h>

// #include <mmsystem.h>
// #pragma comment(lib, "winmm.lib")

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"
#include "sleep_ms.h"
#include "dsp/dsp_windows.h"

// OpenCV Includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 


#define BUILD_BLADERF

// Project Includes
//#include <bladerf_common.h>
#include "bladerf_sdr.h"


const std::complex<double> j = std::complex<double>(0, 1);
const double pi = 3.14159265358979;

//-----------------------------------------------------------------------------
template<class T, class U>
inline std::complex<T> complex_cast(const std::complex<U>& c) {

    return std::complex<T>(std::real(c), std::imag(c));
}


std::unique_ptr<SDR_BASE> SDR_BASE::build()
{
#ifdef BUILD_BLADERF 
    std::unique_ptr<BLADERF_SDR> bladerf_dev = BLADERF_SDR::open();

    // Use sample rate if set, otherwise default to 2.4MSPS.

    bladerf_dev->init_rx();

    //if (config.Bladerf.sampleRate != 0) {
    //    bladerf_dev->setSampleRate(config.Bladerf.sampleRate);
    //}
    //else {
    //    bladerf_dev->setSampleRate(2400000);
    //}

    bladerf_dev->set_rx_frequency(96700000);
    bladerf_dev->set_rx_samplerate(624000);
    bladerf_dev->set_rx_gain(30, BLADERF_GAIN_MANUAL);
    bladerf_dev->set_rx_bandwidth(1000000);

    //bladerf_dev->setSamplePublisher(std::move(config.Bladerf.samplePublisher));
    return std::unique_ptr<SDR_BASE>(bladerf_dev.release());
#endif

}





//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{

    uint32_t idx;
    
    uint64_t sample_rate;

    uint32_t num_samples;

    const float scale = (1.0 / 2048.0);

    // number of taps to create a low pass RF filter
    uint64_t n_taps = 100;

    // offset from the center where we want to demodulate(Hz)
    int64_t f_offset = 115750;

    // rf frequency filter cutoff
    int64_t fc_rf = 62400;

    // the FM broadcast signal has a bandwidth(Hz)
    int64_t desired_rf_sample_rate = fc_rf * 2;

    // audio sample rate ==> 5 times the data bit rate
    int64_t bit_rate = 4160;
    int64_t desired_audio_sample_rate = 5 * bit_rate;

    // audio filter cutoff frequency(Hz)
    int64_t fc_audio = 2400 * 3;

    try{

        std::unique_ptr<SDR_BASE> sdr = SDR_BASE::build();

        sample_rate = sdr->get_rx_samplerate();

        // number of samples is equal to the number seconds to record times the samplerate
        num_samples = 15 * 3600 * sample_rate;

        // decimation factor
        int64_t rf_decimation_factor = (int64_t)(sample_rate / (float)desired_rf_sample_rate);

        // calculate the new sampling rate based on the original and the decimated sample rate
        float decimated_sample_rate = sample_rate / (float)rf_decimation_factor;

        // build the decimation sequence
        //af::array dec_seq = af::seq(0, num_samples, dec_rate);

        // low pass filter coefficients
        std::vector<float> lpf_rf = DSP::create_fir_filter(n_taps, (desired_rf_sample_rate / 2.0) / (float)sample_rate, &DSP::hann_window);
        // create the low pass filter from the filter coefficients
        //af::array af_lpf = af::array(lpf.size(), (float*)lpf.data());

        // find a decimation rate to achieve audio sampling rate between for 10 kHz
        int64_t audio_decimation_factor = (int64_t)(decimated_sample_rate / (float)desired_audio_sample_rate);
        float decimated_audio_sample_rate = decimated_sample_rate / (float)audio_decimation_factor;

        // scaling for tangent
        float phasor_scale = 1 / ((2 * M_PI) / (decimated_sample_rate / desired_rf_sample_rate));

        std::vector<float> lpf_fm = DSP::create_fir_filter(64, 1.0/(float)(decimated_sample_rate * 75e-6), &DSP::rectangular_window);
        //af::array af_lpf_de = af::array(lpf_de.size(), (float*)lpf_de.data());

        std::vector<float> lpf_audio = DSP::create_fir_filter(n_taps, (desired_audio_sample_rate / 2.0) / (float)decimated_sample_rate, &DSP::hann_window);
        //af::array af_lpf_a = af::array(lpf_a.size(), (float*)lpf_a.data());

        // A*exp(j*3*pi*t) = A*cos(3*pi*t) + j*sin(3*pi*t)
        // generate the frequency rotation vector to center the offset frequency 
        std::vector<std::complex<float>> fc_rot(num_samples, 0);
        for (idx = 0; idx < num_samples; ++idx)
        {
            fc_rot[idx] = std::exp(-2.0 * 1i * M_PI * (f_offset / (double)sample_rate) * (double)idx);
        }

        //std::vector<std::complex<float>> cf_samples(num_samples);
        std::complex<float> cf_scale(1.0/2048.0f, 0.0f);

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
         std::cout << "fs:         " << sample_rate << std::endl;
         std::cout << "rx_freq:    " << sdr->get_rx_frequency() << std::endl;
         std::cout << "f_offset:   " << f_offset << std::endl;
         std::cout << "channel_bw: " << desired_rf_sample_rate << std::endl;
         std::cout << "dec_rate:   " << rf_decimation_factor << std::endl;
         std::cout << "fs_d:       " << decimated_sample_rate << std::endl;
         std::cout << "audio_freq: " << desired_audio_sample_rate << std::endl;
         std::cout << "dec_audio:  " << audio_decimation_factor << std::endl;
         std::cout << "fs_audio:   " << decimated_audio_sample_rate << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;

        std::vector<std::complex<float>> cf_samples(num_samples);
               
        sdr->start_single(cf_samples, num_samples);
        //sdr->wait_for_samples();

#ifdef USE_ARRAYFIRE
        // take the complex float vector data and put it into an af::array container 
        x2 = af::array(num_samples, (af::cfloat*)cf_samples.data());

        // apply low pass filter to the rotated signal 
        x3 = af::fir(af_lpf, x2);

        // decimate the signal
        x4 = x3(dec_seq);

        // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
        x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
        x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;// .as(f32);

        // run the audio through the low pass de-emphasis filter
        x6 = af::fir(af_lpf_de, x5);

        // run the audio through a second low pass filter before decimation
        x6 = af::fir(af_lpf_a, x6);

        // decimate the audio sequence
        x7 = x6(seq_audio);

        // scale the audio from -1 to 1
        x7 = (x7 * (1.0 / (af::max<float>(af::abs(x7)))));

        // shift to 0 to 2 and then scale by 60
        x7 = ((x7+1) * 40).as(af::dtype::u8);


#endif


        sdr->stop();


    }
    catch(std::exception e)
    {
        std::cout << "error: " << e.what() << std::endl;
    }

    
    return 0;
    
}   // end of main

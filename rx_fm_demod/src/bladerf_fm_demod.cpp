// ----------------------------------------------------------------------------------------
#define _CRT_SECURE_NO_WARNINGS

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)
#define _USE_MATH_DEFINES
#define NOMINMAX

//#include <mmsystem.h>
//#pragma comment(lib, "winmm.lib")

#elif defined(__linux__)

#endif
// ArrayFire Includes
#include <arrayfire.h>

#include <cstdint>
#include <cmath>
#include <iostream>
#include <sstream>
#include <complex>

// bladeRF includes
#include <libbladeRF.h>
#include <bladeRF2.h>

#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

// Custom Includes
#include "num2string.h"
#include "get_current_time.h"
#include "file_parser.h"
#include "file_ops.h"
#include "sleep_ms.h"

// Project Includes
#include <bladerf_common.h>

const std::complex<double> j = std::sqrt(std::complex<double>(-1, 0));
const double pi = 3.14159265358979;

//-----------------------------------------------------------------------------

template<class T, class U>
inline std::complex<T> complex_cast(const std::complex<U>& c) {
//    return { reinterpret_cast<T>(c.real()), reinterpret_cast<T>(c.imag()) };
    //return { (T)(c.real()), (T)(c.imag()) };
    return std::complex<T>(std::real(c), std::imag(c));
}


//-----------------------------------------------------------------------------
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
    bladerf_frequency rx_freq = 96650000;// 96600000; //162425000;
    bladerf_sample_rate fs = 1000000;
    bladerf_bandwidth rx_bw = 500000;
    bladerf_gain rx1_gain = 60;
    int64_t span = 500000;

    int64_t f_offset = 50000;
    int64_t fm_bw = 200000; // 200000; //12500;

    //std::vector<int16_t> samples;
    uint32_t num_samples = 65536*4;
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4 * 8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    const float scale = (1.0 / 2048.0);
    std::vector<std::complex<int16_t>> samples(num_samples);

    // low pass filter for the baseband signal
    //std::vector<float> lpf = { 0.00127024396582050, 0.00137196026584047, 0.00155382454344669, 0.00182500460417206, 0.00219340225150352, 0.00266545076341369, 0.00324593602802402, 0.00393784532622661, 
    //    0.00474224725037087, 0.00565820567220786, 0.00668273003138318, 0.00781076352055918, 0.00903521000862572, 0.0103469997842522, 0.0117351934336911, 0.0131871224050103, 
    //    0.0146885640715190, 0.0162239484054129, 0.0177765927232431, 0.0193289603813703, 0.0208629387944472, 0.0223601317339675, 0.0238021605460190, 0.0251709687145840, 
    //    0.0264491240938766, 0.0276201131428993, 0.0286686216178920, 0.0295807964115892, 0.0303444835678173, 0.0309494379394028, 0.0313875004879594, 0.0316527398353473, 
    //    0.0317415553562104, 0.0316527398353473, 0.0313875004879594, 0.0309494379394028, 0.0303444835678173, 0.0295807964115892, 0.0286686216178920, 0.0276201131428993, 
    //    0.0264491240938766, 0.0251709687145840, 0.0238021605460190, 0.0223601317339675, 0.0208629387944472, 0.0193289603813703, 0.0177765927232431, 0.0162239484054129, 
    //    0.0146885640715190, 0.0131871224050103, 0.0117351934336911, 0.0103469997842522, 0.00903521000862572, 0.00781076352055918, 0.00668273003138318, 0.00565820567220786, 
    //    0.00474224725037087, 0.00393784532622661, 0.00324593602802402, 0.00266545076341369, 0.00219340225150352, 0.00182500460417206, 0.00155382454344669, 0.00137196026584047, 
    //    0.00127024396582050 };

    std::vector<float> lpf = {-6.61391669479832e-05, 0.000335776248411655, 0.000738724068401829, 0.00107649528540311, 0.00123836524062450, 0.00108559941155010, 0.000502275785786987, -0.000532606005849298,
        -0.00187959214747095, -0.00321364757872768, -0.00406622534529712, -0.00394155242196394, -0.00248679264987190, 0.000330411945956007, 0.00409870240775244, 0.00795248761388078,
        0.0107085993685898, 0.0111442478686612, 0.00836416411377399, 0.00216646192124981, -0.00670051344143861, -0.0164859067235112, -0.0246421911404170, -0.0282793387758380,
        -0.0247886033534137, -0.0125015786429992, 0.00877525039518713, 0.0374957649923735, 0.0704707975115531, 0.103335974122769, 0.131311110339503, 0.150096733759403,
        0.156713489985832, 0.150096733759403, 0.131311110339503, 0.103335974122769, 0.0704707975115531, 0.0374957649923735, 0.00877525039518713, -0.0125015786429992,
        -0.0247886033534137, -0.0282793387758380, -0.0246421911404170, -0.0164859067235112, -0.00670051344143861, 0.00216646192124981, 0.00836416411377399, 0.0111442478686612,
        0.0107085993685898, 0.00795248761388078, 0.00409870240775244, 0.000330411945956007, -0.00248679264987190, -0.00394155242196394, -0.00406622534529712, -0.00321364757872768,
        -0.00187959214747095, -0.000532606005849298, 0.000502275785786987, 0.00108559941155010, 0.00123836524062450, 0.00107649528540311, 0.000738724068401829, 0.000335776248411655, -6.61391669479832e-05};

#ifdef USE_ARRAYFIRE
    
    af::setBackend(AF_BACKEND_CPU);
    af::info();

    std::cout << std::endl << std::endl;

    // array fire variables
    af::array raw_data, fft_data, raw_data2;

    af::array x2, x3, x4, x5, x6, x7, x8;

#endif // USE_ARRAYFIRE

    int num_devices = bladerf_get_device_list(&device_list);

    bladerf_num = select_bladerf(num_devices, device_list);

    if (bladerf_num < 0)
    {
        std::cout << "could not detect any bladeRF devices..." << std::endl;
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
        blade_status = bladerf_set_frequency(dev, rx, rx_freq);
        blade_status = bladerf_set_sample_rate(dev, rx, fs, &fs);
        blade_status = bladerf_set_bandwidth(dev, rx, rx_bw, &rx_bw);

        // the gain 
        blade_status = bladerf_set_gain_mode(dev, rx, BLADERF_GAIN_MANUAL);
        blade_status = bladerf_set_gain(dev, rx, rx1_gain);

        // configure the sync to receive/transmit data
        blade_status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11, num_buffers, buffer_size, num_transfers, timeout_ms);

        // enable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, true);

        // decimation rate
        int64_t dec_rate = (int64_t)(fs / (float)fm_bw);

        // calculate the new sampling rate based on the original and the decimated sample rate
        float fs_d = fs / (float)dec_rate;

        // build the decimation sequence
        af::array dec_seq = af::seq(0, num_samples, dec_rate);

        // create the low pass filter from the filter coefficients
        af::array af_lpf = af::array(lpf.size(), (float*)lpf.data());

        // find a decimation rate to achieve audio sampling rate between for 10 kHz
        float audio_freq = 44000;
        int64_t dec_audio = (int64_t)(fs_d / audio_freq);
        float fs_audio = fs_d / (float)dec_audio;

        // scaling for tangent
        float phasor_scale = 1 / ((2 * M_PI) / (fs_d / fm_bw));

        // audio decimation sequence
        af::array seq_audio = af::seq(0, dec_seq.dims(0), dec_audio);

        af::array audio_filter = af::constant(0.5f, 2); // af::array(2, { 0.5f, 0.5f });

        // A*exp(j*3*pi*t) = A*cos(3*pi*t) + j*sin(3*pi*t)

        //af::array test = af::range(af::dim4(num_samples));
        //af::array af_lpf = af::exp(-2.0 * j * pi * (f_offset / (double)sample_rate) * test);

        //af::array af_rot = af::complex(af::cos(-2.0 * pi * (f_offset / (double)fs) * test), af::sin(-2.0 * pi * (f_offset / (double)fs) * test));

        // generate the frequency rotation vector to center the offset frequency 
        std::vector<std::complex<float>> fc_rot(num_samples, 0);
        for (idx = 0; idx < num_samples; ++idx)
        {
            //fc_rot[idx] = std::complex<float>(cos(-2.0 * pi * (f_offset / (double)sample_rate) * idx), sin(-2.0 * pi * (f_offset / (double)sample_rate) * idx));
            fc_rot[idx] = std::exp(-2.0 * 1i * M_PI * (f_offset / (double)fs) * (double)idx);
        }

        double freq_step = (fs)/(double)num_samples;

        double f_min = (rx_freq - (span>>1)) * 1.0e-6;
        double f_max = (rx_freq + (span>>1)) * 1.0e-6;

        uint32_t sp = (uint32_t)((fs - span) / (2.0 * freq_step));
        uint32_t sp2 = (uint32_t)(span / freq_step);

        double fft_scale = 1.0 / (double)(num_samples);

        // try to compute the fft using the arrayfire library
        std::vector<std::complex<float>> cf_samples(num_samples);
        std::complex<float> cf_scale(scale, scale);

#if defined(_WIN32) | defined(__WIN32__) | defined(__WIN32) | defined(_WIN64) | defined(__WIN64)

        // audio setup for windows only
        HWAVEOUT hWaveOut = 0;
        WAVEFORMATEX wfx = { WAVE_FORMAT_PCM, 1, (DWORD)fs_audio, (DWORD)fs_audio, 1, 8, 0 };
        waveOutOpen(&hWaveOut, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL);

#endif


#ifdef USE_ARRAYFIRE

        af::Window myWindow(800, 800, "FFT example: ArrayFire");
        //af::array X = af::seq(0, num_samples - 1, 1);
        af::array X = af::seq(sp+1, (sp+sp2), 1);
        //auto x2d = X.dims(0);

        af::array f = af::seq(f_min, f_max - (freq_step*1.0e-6), (freq_step*1.0e-6));

        myWindow.setAxesLimits(f_min, f_max - (freq_step * 1.0e-6), -120, -20, true);
        myWindow.setAxesTitles("Frequency (MHz)", "Power (dBm)");
//        myWindow.setAxesLimits(-0.5, 0.5, -0.5, 0.5, false);
//        myWindow.setAxesTitles("Real", "Imag");

        while (!myWindow.close())

#else

        while(1)
#endif

        {
            blade_status = bladerf_sync_rx(dev, (void*)samples.data(), num_samples, NULL, timeout_ms);
            if (blade_status != 0)
            {
                std::cout << "Unable to get the required number of samples: " << std::string(bladerf_strerror(blade_status)) << std::endl;
                return blade_status;
            }

            // convert from complex int16 to complex floats, scale and frequency shift
            for (idx = 0; idx < num_samples; ++idx)
            {
                cf_samples[idx] = complex_cast<float>(samples[idx]) * fc_rot[idx] * cf_scale;
            }

#ifdef USE_ARRAYFIRE
            // take the complex float vector data and put it into an af::array container 
            x2 = af::array(num_samples, (af::cfloat*)cf_samples.data());

            // apply low pass filter to the rotated signal 
            x3 = af::fir(af_lpf, x2);
            //x3 = af::fir(af_lpf, x3);

            // decimate the signal
            x4 = x3(dec_seq);

            // polar discriminator - x4(2:end).*conj(x4(1:end - 1));
            x5 = x4(af::seq(1, af::end, 1)) * af::conjg(x4(af::seq(0, -2, 1)));
            x5 = af::atan2(af::imag(x5), af::real(x5)) * phasor_scale;// .as(f32);


            //af::array t2 = x4(af::seq(1, af::end, 1));
            //af::array t3 = x4(af::seq(0, -2, 1));

            //auto t4 = t2.dims();
            //auto t5 = t3.dims();

            //auto t1 = x5.type();
            //auto t1a = x5.dims();

            // a = [1, -1/3], b = 2/3
            x6 = af::fir(audio_filter, x5);
            //x6 = af::fir(audio_filter, x6);

            // decimate the audio sequence
            x7 = x6(seq_audio);
            //x7 = (x7 * (127.0 / (af::max<float>(x7)))).as(af::dtype::u8);
            x7 = (x7 * 16.0).as(af::dtype::u8);

            WAVEHDR header = { (LPSTR)x7.host<uint8_t>(), x7.dims(0)*sizeof(uint8_t), 0, 0, 0, 0, 0, 0 };

            waveOutPrepareHeader(hWaveOut, &header, sizeof(WAVEHDR));
            waveOutWrite(hWaveOut, &header, sizeof(WAVEHDR));

//            sleep_ms(100);

            //af::fftInPlace(raw_data, scale);

            fft_data = 20 * af::log10(af::shift(af::abs(af::fft(x3)*fft_scale), (num_samples >> 1)))-10;

            //// show the results of the FFT in the window
            myWindow.plot(f, fft_data(X));
            //myWindow.scatter(af::real(x4), af::imag(x4));
            myWindow.show();
#endif
        }

        // disable the rx channel RF frontend
        blade_status = bladerf_enable_module(dev, BLADERF_RX, false);

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

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
std::vector<float> blackman_window(uint64_t N)
{
    std::vector<float> w(N, 0);

    for (uint64_t idx = 0; idx < N; ++idx)
    {
        w[idx] = 0.42 - 0.5 * std::cos(2* M_PI * idx / (N - 1)) + 0.08 * std::cos(4 * M_PI * idx / (N - 1));
    }

    return w;
}   // end of blackman_window

//-----------------------------------------------------------------------------
std::vector<float> blackman_fir(uint64_t N, float fc)
{
    std::vector<float> g(N, 0);

    std::vector<float> w = blackman_window(N);

    for (uint64_t idx = 0; idx < N; ++idx)
    {
        if (abs((double)idx -  (N / 2.0)) < 1e-6)
            g[idx] = w[idx] * fc;
        else
            g[idx] = w[idx] * (std::sin(M_PI * fc * (idx - N / 2.0)) / (M_PI * (idx - N / 2.0)));
    }

    return g;

}   // end of blackman_fir



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
    bladerf_frequency rx_freq = 162400000;// 96600000; //162400000;
    bladerf_sample_rate fs = 1000000;
    bladerf_bandwidth rx_bw = 500000;
    bladerf_gain rx1_gain = 60;
    int64_t span = 100000;

    int64_t f_offset = 50000;
    int64_t channel_bw = 25000; // 200000; //25000;
    int64_t audio_freq = 20000;

    //std::vector<int16_t> samples;
    uint32_t num_samples = 65536*4;
    uint32_t timeout_ms = 10000;
    const uint32_t num_buffers = 16;
    const uint32_t buffer_size = 1024 * 4 * 8;        // must be a multiple of 1024
    const uint32_t num_transfers = 8;
    const float scale = (1.0 / 2048.0);
    std::vector<std::complex<int16_t>> samples(num_samples);

    // low pass filter for the baseband signal
    std::vector<float> lpf = { 
    // weather radio
        -0.000614022815872799, -0.000569546482030748, -0.000534906718409012, -0.000494792778957206, -0.000430138046170730, -0.000318737140452114, -0.000136044581405287, 0.000143872014986503,
        0.000547258612146006, 0.00109952182911627, 0.00182404150604869, 0.00274099663046851, 0.00386625335013228, 0.00521036265378273, 0.00677771217045058, 0.00856587151743979,
        0.0105651638935747, 0.0127584884184495, 0.0151214083752485, 0.0176225103907219, 0.0202240290843108, 0.0228827212625181, 0.0255509637502631, 0.0281780398487146,
        0.0307115715670181, 0.0330990485229106, 0.0352894000113257, 0.0372345543934945, 0.0388909297719475, 0.0402208019118495, 0.0411934994799879, 0.0417863827466676,
        0.0419855696994477, 0.0417863827466676, 0.0411934994799879, 0.0402208019118495, 0.0388909297719475, 0.0372345543934945, 0.0352894000113257, 0.0330990485229106,
        0.0307115715670181, 0.0281780398487146, 0.0255509637502631, 0.0228827212625181, 0.0202240290843108, 0.0176225103907219, 0.0151214083752485, 0.0127584884184495,
        0.0105651638935747, 0.00856587151743979, 0.00677771217045058, 0.00521036265378273, 0.00386625335013228, 0.00274099663046851, 0.00182404150604869, 0.00109952182911627,
        0.000547258612146006, 0.000143872014986503, -0.000136044581405287, -0.000318737140452114, -0.000430138046170730, -0.000494792778957206, -0.000534906718409012, -0.000569546482030748, -0.000614022815872799

        //0.00127024396582050, 0.00137196026584047, 0.00155382454344669, 0.00182500460417206, 0.00219340225150352, 0.00266545076341369, 0.00324593602802402, 0.00393784532622661, 
        //0.00474224725037087, 0.00565820567220786, 0.00668273003138318, 0.00781076352055918, 0.00903521000862572, 0.0103469997842522, 0.0117351934336911, 0.0131871224050103, 
        //0.0146885640715190, 0.0162239484054129, 0.0177765927232431, 0.0193289603813703, 0.0208629387944472, 0.0223601317339675, 0.0238021605460190, 0.0251709687145840, 
        //0.0264491240938766, 0.0276201131428993, 0.0286686216178920, 0.0295807964115892, 0.0303444835678173, 0.0309494379394028, 0.0313875004879594, 0.0316527398353473, 
        //0.0317415553562104, 0.0316527398353473, 0.0313875004879594, 0.0309494379394028, 0.0303444835678173, 0.0295807964115892, 0.0286686216178920, 0.0276201131428993, 
        //0.0264491240938766, 0.0251709687145840, 0.0238021605460190, 0.0223601317339675, 0.0208629387944472, 0.0193289603813703, 0.0177765927232431, 0.0162239484054129, 
        //0.0146885640715190, 0.0131871224050103, 0.0117351934336911, 0.0103469997842522, 0.00903521000862572, 0.00781076352055918, 0.00668273003138318, 0.00565820567220786, 
        //0.00474224725037087, 0.00393784532622661, 0.00324593602802402, 0.00266545076341369, 0.00219340225150352, 0.00182500460417206, 0.00155382454344669, 0.00137196026584047, 0.00127024396582050

    // normal radio
        //0.00119262903445878, 0.00129604397835963, 0.00147609249714922, 0.00174264515305833, 0.00210435428268565, 0.00256842922124053, 0.00314043494341142, 0.00382411870233896,
        //0.00462126873438364, 0.00553160849147962, 0.00655272917692122, 0.00768006260833508, 0.00890689562878912, 0.0102244264501376, 0.0116218624593147, 0.0130865581663653,
        //0.0146041911406339, 0.0161589729864995, 0.0177338916694565, 0.0193109808332078, 0.0208716111633791, 0.0223967983663343, 0.0238675219531958, 0.0252650487581053,
        //0.0265712549820771, 0.0277689405429569, 0.0288421296287847, 0.0297763515943123, 0.0305588967039088, 0.0311790417013726, 0.0316282407685828, 0.0319002781085543,
        //0.0319913791404192, 0.0319002781085543, 0.0316282407685828, 0.0311790417013726, 0.0305588967039088, 0.0297763515943123, 0.0288421296287847, 0.0277689405429569,
        //0.0265712549820771, 0.0252650487581053, 0.0238675219531958, 0.0223967983663343, 0.0208716111633791, 0.0193109808332078, 0.0177338916694565, 0.0161589729864995,
        //0.0146041911406339, 0.0130865581663653, 0.0116218624593147, 0.0102244264501376, 0.00890689562878912, 0.00768006260833508, 0.00655272917692122, 0.00553160849147962,
        //0.00462126873438364, 0.00382411870233896, 0.00314043494341142, 0.00256842922124053, 0.00210435428268565, 0.00174264515305833, 0.00147609249714922, 0.00129604397835963, 0.00119262903445878
    };

    std::vector<float> lpf2 = blackman_fir(64, (channel_bw / 2.0) / (float)fs);

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
        int64_t dec_rate = (int64_t)(fs / (float)channel_bw);

        // calculate the new sampling rate based on the original and the decimated sample rate
        float fs_d = fs / (float)dec_rate;

        // build the decimation sequence
        af::array dec_seq = af::seq(0, num_samples, dec_rate);

        // create the low pass filter from the filter coefficients
        af::array af_lpf = af::array(lpf.size(), (float*)lpf.data());

        // find a decimation rate to achieve audio sampling rate between for 10 kHz
        int64_t dec_audio = (int64_t)(fs_d / (float)audio_freq);
        float fs_audio = fs_d / (float)dec_audio;

        // scaling for tangent
        float phasor_scale = 1 / ((2 * M_PI) / (fs_d / channel_bw));

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

        // print out the specifics
        std::cout << std::endl << "------------------------------------------------------------------" << std::endl;
        std::cout << "fs:         " << fs << std::endl;
        std::cout << "rx_freq:    " << rx_freq << std::endl;
        std::cout << "f_offset:   " << f_offset << std::endl;
        std::cout << "channel_bw: " << channel_bw << std::endl;
        std::cout << "dec_rate:   " << dec_rate << std::endl;
        std::cout << "fs_d:       " << fs_d << std::endl;
        std::cout << "audio_freq: " << audio_freq << std::endl;
        std::cout << "dec_audio:  " << dec_audio << std::endl;
        std::cout << "fs_audio:   " << fs_audio << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl << std::endl;


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

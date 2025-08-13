/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>


/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// @authors: Leonard Hall <LeonardTHall@gmail.com>, template implmentation: Daniel Frenzel <dgdanielf@gmail.com>
template <class T>
class DigitalBiquadFilter {
public:
    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };

    CLASS_NO_COPY(DigitalBiquadFilter);

    DigitalBiquadFilter();

    T apply(const T &sample, const struct biquad_params &params);
    void reset();
    void reset(const T &value, const struct biquad_params &params);
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
    
private:
    T _delay_element_1;
    T _delay_element_2;
    bool initialised;
};

template <class T>
class LowPassFilter2p {
public:
    LowPassFilter2p();
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    // return the cutoff frequency
    float get_cutoff_freq(void) const;
    float get_sample_freq(void) const;
    T apply(const T &sample);
    void reset(void);
    void reset(const T &value);

    CLASS_NO_COPY(LowPassFilter2p);

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;
    
private:
    DigitalBiquadFilter<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter2p<T>::LowPassFilter2p() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LowPassFilter2p<T>::LowPassFilter2p(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}
*/

typedef LowPassFilter2p<int>      LowPassFilter2pInt;
typedef LowPassFilter2p<long>     LowPassFilter2pLong;
typedef LowPassFilter2p<float>    LowPassFilter2pFloat;
typedef LowPassFilter2p<Vector2f> LowPassFilter2pVector2f;
typedef LowPassFilter2p<Vector3f> LowPassFilter2pVector3f;



////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilterMp
////////////////////////////////////////////////////////////////////////////////////////////
// ToDo:
// - Check if GCS_SEND_TEXT has an text length limit!
// - Init function could return the filter order, and the AP Param could be overwritten!
// - Naming of Parameters: INS_ML_GYR_NTCH -> INS_ML_GYR_HNTCH, INS_ML_NTCH_ENAB
// - Checks: Call of apply() in unitialized state; Call of apply with cutoff frequency to zero
// - Make sure, that filters are applied in order that minimates the clipping risk (filters with gain > 1 last)
// - One could precalculate analog prototype poles in the init function

#include "complexf.h"
//#include <AP_Math/AP_Math.h> -> for M_PI (already included)
//#include <cmath> -> for trigonometric functions (already included)

#define LPF_MP_MAX_FILTERS 3 // The cascade consists of biquads, so the maximum filter order will be 2*LPF_MP_MAX_FILTERS.

template <class T>
class LowPassFilterMp {
public:
    enum FilterType {
        Butterworth = 1,
        PTn,
        Bessel
    };

    // constructor
    LowPassFilterMp();
    //LowPassFilterMp(float sample_freq, float cutoff_freq);
    // destructor
    ~LowPassFilterMp();

    // initialize filters
    void init(uint8_t filter_order, uint8_t filter_type);
    
    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    
    // return the cutoff frequency and sample frequency
    float get_cutoff_freq(void) const;
    float get_sample_freq(void) const;
    
    T apply(const T &sample);
    
    void reset(void);
    //void reset(const T &value);

    CLASS_NO_COPY(LowPassFilterMp);

protected:
    // biquad params
    typename DigitalBiquadFilter<T>::biquad_params* _params;
    
private:
    // biquad filters
    DigitalBiquadFilter<T>* _filters;

    // number of allocated filters
    uint8_t _num_filters {};
    uint8_t _filter_order {};
    uint8_t _filter_type  {};

    // filter settings
    float _sample_freq {};
    float _cutoff_freq {};

    void allocate_filters(uint8_t num_filters);

    // calculates the biquad coefficients
    void compute_params(void);

    // functions for analog filter prototypes
    void compute_butterworth_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]);
    void compute_ptn_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]);
    void compute_bessel_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]);
};

typedef LowPassFilterMp<float>    LowPassFilterMpFloat;
typedef LowPassFilterMp<Vector3f> LowPassFilterMpVector3f;

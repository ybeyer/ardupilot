#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif

#include "LowPassFilter2p.h"
#include <GCS_MAVLink/GCS.h>


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalBiquadFilter
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalBiquadFilter<T>::DigitalBiquadFilter() {
  _delay_element_1 = T();
  _delay_element_2 = T();
}

template <class T>
T DigitalBiquadFilter<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    if (!initialised) {
        reset(sample, params);
        initialised = true;
    }

    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

template <class T>
void DigitalBiquadFilter<T>::reset() { 
    _delay_element_1 = _delay_element_2 = T();
    initialised = false;
}

template <class T>
void DigitalBiquadFilter<T>::reset(const T &value, const struct biquad_params &params) {
    _delay_element_1 = _delay_element_2 = value * (1.0 / (1 + params.a1 + params.a2));
    initialised = true;
}

template <class T>
void DigitalBiquadFilter<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    ret.b0 = ohm*ohm/c;
    ret.b1 = 2.0f*ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret.a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
}


////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter2p
////////////////////////////////////////////////////////////////////////////////////////////

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

// change parameters
template <class T>
void LowPassFilter2p<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    DigitalBiquadFilter<T>::compute_params(sample_freq, cutoff_freq, _params);
}

// return the cutoff frequency
template <class T>
float LowPassFilter2p<T>::get_cutoff_freq(void) const {
    return _params.cutoff_freq;
}

template <class T>
float LowPassFilter2p<T>::get_sample_freq(void) const {
    return _params.sample_freq;
}

template <class T>
T LowPassFilter2p<T>::apply(const T &sample) {
    if (!is_positive(_params.cutoff_freq)) {
        // zero cutoff means pass-thru
        return sample;
    }
    return _filter.apply(sample, _params);
}

template <class T>
void LowPassFilter2p<T>::reset(void) {
    return _filter.reset();
}

template <class T>
void LowPassFilter2p<T>::reset(const T &value) {
    return _filter.reset(value, _params);
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter2p<int>;
template class LowPassFilter2p<long>;
template class LowPassFilter2p<float>;
template class LowPassFilter2p<Vector2f>;
template class LowPassFilter2p<Vector3f>;



////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilterMp
////////////////////////////////////////////////////////////////////////////////////////////
template <class T>
LowPassFilterMp<T>::LowPassFilterMp()
    : _filters {nullptr}
    , _params {nullptr}
{}


/*
template <class T>
LowPassFilterMp<T>::LowPassFilterMp(float sample_freq, float cutoff_freq, uint8_t num_filters) {
    allocate_filters(num_filters);
    set_cutoff_frequency(sample_freq, cutoff_freq);
}
*/


template <class T>
void LowPassFilterMp<T>::init(uint8_t filter_order, uint8_t filter_type)
{
    if (filter_order < 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LowPassFilterMp: Requested filter order must be at least one (desired: %u)", filter_order);
        _filter_order = 1;
    }
    else if (filter_order > 2*LPF_MP_MAX_FILTERS) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LowPassFilterMp: Requested filter order too high (desired: %u, max: %u), using the maximum instead", filter_order, static_cast<uint8_t>(2*LPF_MP_MAX_FILTERS));
        _filter_order = 2*LPF_MP_MAX_FILTERS;
    }
    else {
        _filter_order = filter_order;
    }

    allocate_filters((_filter_order + 1) / 2);

    // ToDo: Check for valid range?
    _filter_type = filter_type;
}


// allocate a collection of biquad filters
template <class T>
void LowPassFilterMp<T>::allocate_filters(uint8_t num_filters)
{
    _num_filters = num_filters;
    if (_num_filters > 0) {
        _filters = new DigitalBiquadFilter<T>[_num_filters];
        _params = new typename DigitalBiquadFilter<T>::biquad_params[_num_filters];
        if (_filters == nullptr || _params == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LowPassFilterMp: Failed to allocate %u bytes for the filter cascade", (unsigned int)(_num_filters * (sizeof(DigitalBiquadFilter<T>) + sizeof(typename DigitalBiquadFilter<T>::biquad_params)))); // ToDo: Check, that this line is correct (usage of typename)
            _num_filters = 0;
        }
    }
}


// destroy all of the associated biquad filters
template <class T>
LowPassFilterMp<T>::~LowPassFilterMp() {
    delete[] _filters;
    delete[] _params;
    _num_filters = 0;
}


// apply a sample to each of the underlying filters in turn and return the output
template <class T>
T LowPassFilterMp<T>::apply(const T &sample) {
    T output = sample;
    for (uint8_t i = 0; i < _num_filters; ++i) {
        // ToDo: Do we really need this check? The biquad also applies checks...
        if (!is_positive(_params[i].cutoff_freq)) {
            // zero cutoff means pass-thru
            continue;
        }
        output = _filters[i].apply(output, _params[i]);
    }
    return output;
}


// change parameters
template <class T>
void LowPassFilterMp<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    _sample_freq = sample_freq;
    _cutoff_freq = cutoff_freq;

    compute_params();
}


// return the cutoff frequency
template <class T>
float LowPassFilterMp<T>::get_cutoff_freq(void) const {
    return _cutoff_freq;
}


// return the sample frequency
template <class T>
float LowPassFilterMp<T>::get_sample_freq(void) const {
    return _sample_freq;
}


template <class T>
void LowPassFilterMp<T>::reset(void) {
    for (uint8_t i = 0; i < _num_filters; ++i) {
        _filters[i].reset();
    }
}

/*
template <class T>
void LowPassFilter2p<T>::reset(const T &value) {
    return _filter.reset(value, _params);
}
*/


// Calculate the cascaded biquad filters coefficients
template <class T>
void LowPassFilterMp<T>::compute_params(void) {
    
    ComplexF poles[LPF_MP_MAX_FILTERS] {};
    
    // ToDo: Calculation of analog prototype could be done in the init function if filter type is a reboot parameter! -> Reduce computational load
    switch (_filter_type) {
        case 1:
            compute_butterworth_analog(poles);
            break;
        case 2:
            compute_ptn_analog(poles);
            break;
        case 3:
            compute_bessel_analog(poles);
            break;
        default:
            compute_butterworth_analog(poles);
    }

    // scale poles to cutoff frequency with frequency pre-warp for bilinear z-transform
    float scale = 2.0F * _sample_freq * std::tan(M_PI*_cutoff_freq/_sample_freq);
    for (uint8_t idx = 0; idx < _num_filters; ++idx) {
        poles[idx] *= scale;
    }

    // transform poles into z-plane through bilinear transform
    for (uint8_t idx = 0; idx < _num_filters; ++idx) {
        poles[idx] = ( 1.0F + poles[idx] / (2.0F*_sample_freq) ) / ( 1.0F - poles[idx] / (2.0F*_sample_freq) );
    }

    // calculate coefficients
    
    // second order element(s)
    uint8_t idx_max = (_filter_order % 2) ? _num_filters - 1 : _num_filters;
    for (uint8_t idx = 0; idx < idx_max; ++idx) {
        float a1, a2, k;

        a1 = -2.0F*poles[idx].real();
        a2 = poles[idx].real() * poles[idx].real() + poles[idx].imag() * poles[idx].imag();

        k = (1.0F + a1 + a2) / 4.0F;

        _params[idx].cutoff_freq = _cutoff_freq;    // value is incorrect for this kind of parametrization, but must be positive for DigitaBiquadFilter to work!
        _params[idx].sample_freq = _sample_freq;    // value is incorrect for this kind of parametrization, but must be positive for DigitaBiquadFilter to work!
        _params[idx].a1 = a1;
        _params[idx].a2 = a2;
        _params[idx].b0 = 1.0F*k;   // we could save some computation time here and in the biquad filter through directly using k, but this requires a change in DigitalBiquadFilter
        _params[idx].b1 = 2.0F*k;
        _params[idx].b2 = 1.0F*k;
    }

    // first order element (for odd filter orders)
    if (_filter_order % 2) {
        float a1, a2, k;

        a1 = -1.0F*poles[_num_filters-1].real(); // This pole is already a real pole
        a2 = 0.0F;

        k = (1.0F + a1 + a2) / 2.0F;

        _params[_num_filters-1].cutoff_freq = _cutoff_freq;    // value is incorrect for this kind of parametrization, but must be positive for DigitaBiquadFilter to work!
        _params[_num_filters-1].sample_freq = _sample_freq;    // value is incorrect for this kind of parametrization, but must be positive for DigitaBiquadFilter to work!
        _params[_num_filters-1].a1 = a1;
        _params[_num_filters-1].a2 = a2;
        _params[_num_filters-1].b0 = 1.0F*k;   // we could save some computation time here and in the biquad filter through directly using k, but this requires a change in DigitalBiquadFilter
        _params[_num_filters-1].b1 = 1.0F*k;
        _params[_num_filters-1].b2 = 0.0F;
    }
}


template <class T>
void LowPassFilterMp<T>::compute_butterworth_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]) {
    
    // Calculate the reduced poles set of the analog filter prototype with cutoff at 1 rad/s
    for (uint8_t k = 0; k < _num_filters; ++k) {
        float theta = static_cast<float>(2 * k + 1) * M_PI / (2 * _filter_order);
        float real = -std::sin(theta);
        float imag =  std::cos(theta);
        poles[k] = ComplexF(real, imag);
    }
}


template <class T>
void LowPassFilterMp<T>::compute_ptn_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]) {

    // Calculate the reduced poles set of the analog filter prototype with cutoff at 1 rad/s

    // calculate the time constant
    static constexpr float gain = 1.0/std::sqrt(2);          // -3.01 dB
    float gain_per_pt1 = std::pow(gain, 1.0F/_filter_order); // gain per PT1 stage (gain must be splitted between _filter_order stages)
    float time_constant = std::sqrt(1.0F/(gain_per_pt1*gain_per_pt1) - 1.0F);

    auto pole = ComplexF(-1.0F/time_constant, 0.0F);

    for (uint8_t idx = 0; idx < _num_filters; ++idx) {
        poles[idx] = pole;
    }    
}


template <class T>
void LowPassFilterMp<T>::compute_bessel_analog(ComplexF (&poles)[LPF_MP_MAX_FILTERS]) {

    // Calculate the reduced poles set of the analog filter prototype with cutoff at 1 rad/s

    // Hardcoded lookup (we do not want to search for the roots of higher order polynomials at runtime)
    //  Calculate with MATLAB or Python, or use precalculated values, e.g. http://www.crbond.com/papers/bsf2.pdf
    switch (_filter_order) {
        case 1:
            poles[0] = ComplexF(-1.0F, 0.0F);
            break;
        case 2:
            poles[0] = ComplexF(-1.1016013306F, 0.6360098248F);
            break;
        case 3:
            poles[0] = ComplexF(-1.0474091610F, 0.9992644363F);
            poles[1] = ComplexF(-1.3226757999F, 0.0F);
            break;
        case 4:
            poles[0] = ComplexF(-0.9952087644F, 1.2571057395F);
            poles[1] = ComplexF(-1.3700678306F, 0.4102497175F);
            break;
        case 5:
            poles[0] = ComplexF(-0.9576765486F, 1.4711243207);
            poles[1] = ComplexF(-1.3808773259F, 0.7179095876F);
            poles[2] = ComplexF(-1.5023162714F, 0.0F);
            break;
        case 6:
            poles[0] = ComplexF(-0.9306565229F, 1.6618632689F);
            poles[1] = ComplexF(-1.3818580976F, 0.9714718907F);
            poles[2] = ComplexF(-1.5714904036F, 0.3208963742F);
            break;
    }

    #if (LPF_MP_MAX_FILTERS > 3)
        #error When increasing LPF_MP_MAX_FILTERS, LowPassFilterMp::compute_bessel_analog() must be adapted!
    #endif 

}


template class LowPassFilterMp<float>;
template class LowPassFilterMp<Vector3f>;

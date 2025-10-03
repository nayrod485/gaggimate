// ImprovedFiltering.h - Enhanced filtering methods for PressureController
#ifndef IMPROVED_FILTERING_H
#define IMPROVED_FILTERING_H

#include <deque>
#include <vector>
#include <cmath>
#include <memory>

/**
 * Base class for derivative filters
 * Provides common interface for different filtering approaches
 */
class DerivativeFilter {
public:
    virtual ~DerivativeFilter() = default;
    
    // Add a sample to the filter
    virtual void addSample(float value, float timestamp = 0.0f) = 0;
    
    // Get the derivative estimate
    virtual float getDerivative(float dt) const = 0;
    
    // Get the filtered value (smoothed signal)
    virtual float getFilteredValue() const = 0;
    
    // Check if filter is ready to provide valid estimates
    virtual bool isReady() const = 0;
    
    // Reset the filter state
    virtual void reset() = 0;
    
    // Get filter type name for debugging
    virtual const char* getTypeName() const = 0;
};

/**
 * Savitzky-Golay Filter for smooth derivative estimation
 * Excellent for pressure derivatives where smoothness is critical
 */
class SavitzkyGolayFilter : public DerivativeFilter {
private:
    std::deque<float> samples;
    std::vector<float> derivative_coeffs;
    std::vector<float> smoothing_coeffs;
    size_t window_size;
    int polynomial_order;
    bool coefficients_initialized;
    float current_timestamp;

public:
    SavitzkyGolayFilter(size_t window_size = 7, int poly_order = 2) 
        : window_size(window_size), polynomial_order(poly_order), 
          coefficients_initialized(false), current_timestamp(0.0f) {
        if (window_size % 2 == 0) window_size++; // Ensure odd window size
        initializeCoefficients();
    }
    
    void addSample(float value, float timestamp = 0.0f) override {
        samples.push_back(value);
        current_timestamp = timestamp;
        
        if (samples.size() > window_size) {
            samples.pop_front();
        }
    }
    
    float getDerivative(float dt) const override {
        if (!coefficients_initialized || samples.size() < window_size) {
            return 0.0f;
        }
        
        // Calculate derivative at the most recent point (rightmost) for real-time control
        // This uses the rightmost coefficients of the Savitzky-Golay filter
        float derivative = 0.0f;
        for (size_t i = 0; i < window_size; ++i) {
            derivative += derivative_coeffs[i] * samples[i];
        }
        return derivative / dt;
    }
    
    // Alternative method for center-point derivative (if needed for analysis)
    float getCenterDerivative(float dt) const {
        if (!coefficients_initialized || samples.size() < window_size) {
            return 0.0f;
        }
        
        // Calculate derivative at the center point of the window
        float derivative = 0.0f;
        for (size_t i = 0; i < window_size; ++i) {
            derivative += derivative_coeffs[i] * samples[i];
        }
        return derivative / dt;
    }
    
    float getFilteredValue() const override {
        if (!coefficients_initialized || samples.size() < window_size) {
            return samples.empty() ? 0.0f : samples.back();
        }
        
        // Center-window filtering: filter the midpoint of the window
        // This reduces lag compared to left-window filtering
        float filtered = 0.0f;
        for (size_t i = 0; i < window_size; ++i) {
            filtered += smoothing_coeffs[i] * samples[i];
        }
        return filtered;
    }
    
    bool isReady() const override { 
        return coefficients_initialized && samples.size() >= window_size; 
    }
    
    
    void reset() override {
        samples.clear();
        current_timestamp = 0.0f;
    }
    
    const char* getTypeName() const override { return "SavitzkyGolay"; }

private:
    // Compute Savitzky-Golay coefficients using least squares method
    void computeSavitzkyGolayCoefficients() {
        int m = (window_size - 1) / 2; // Half window
        int n = polynomial_order;
        
        // Build Vandermonde matrix A where A[i][j] = (i-m)^j
        std::vector<std::vector<float>> A(window_size, std::vector<float>(n + 1));
        for (int i = 0; i < (int)window_size; ++i) {
            int x = i - m; // Position relative to center
            float x_power = 1.0f;
            for (int j = 0; j <= n; ++j) {
                A[i][j] = x_power;
                x_power *= x;
            }
        }
        
        // Compute A^T * A
        std::vector<std::vector<float>> AtA(n + 1, std::vector<float>(n + 1, 0.0f));
        for (int i = 0; i <= n; ++i) {
            for (int j = 0; j <= n; ++j) {
                for (int k = 0; k < (int)window_size; ++k) {
                    AtA[i][j] += A[k][i] * A[k][j];
                }
            }
        }
        
        // Invert (A^T * A) using Gaussian elimination
        std::vector<std::vector<float>> inv_AtA(n + 1, std::vector<float>(n + 1, 0.0f));
        for (int i = 0; i <= n; ++i) inv_AtA[i][i] = 1.0f;
        std::vector<std::vector<float>> work = AtA;
        
        for (int i = 0; i <= n; ++i) {
            // Partial pivoting
            int pivot_row = i;
            float max_val = std::abs(work[i][i]);
            for (int k = i + 1; k <= n; ++k) {
                if (std::abs(work[k][i]) > max_val) {
                    max_val = std::abs(work[k][i]);
                    pivot_row = k;
                }
            }
            if (pivot_row != i) {
                std::swap(work[i], work[pivot_row]);
                std::swap(inv_AtA[i], inv_AtA[pivot_row]);
            }
            
            float pivot = work[i][i];
            if (std::abs(pivot) < 1e-10f) continue;
            
            // Scale pivot row
            for (int j = 0; j <= n; ++j) {
                work[i][j] /= pivot;
                inv_AtA[i][j] /= pivot;
            }
            
            // Eliminate column
            for (int k = 0; k <= n; ++k) {
                if (k != i) {
                    float factor = work[k][i];
                    for (int j = 0; j <= n; ++j) {
                        work[k][j] -= factor * work[i][j];
                        inv_AtA[k][j] -= factor * inv_AtA[i][j];
                    }
                }
            }
        }
        
        // Compute filter coefficients: c = A * (A^T * A)^-1 * b
        // where b is the basis function at evaluation point
        
        // For rightmost point (x = m):
        int x_eval = m;
        
        // Smoothing: b = [1, x_eval, x_eval^2, ...]
        std::vector<float> b_smooth(n + 1);
        float x_power = 1.0f;
        for (int j = 0; j <= n; ++j) {
            b_smooth[j] = x_power;
            x_power *= x_eval;
        }
        
        // Derivative: b = [0, 1, 2*x_eval, 3*x_eval^2, ...]
        std::vector<float> b_deriv(n + 1);
        b_deriv[0] = 0.0f;
        for (int j = 1; j <= n; ++j) {
            x_power = 1.0f;
            for (int p = 0; p < (j - 1); ++p) x_power *= x_eval;
            b_deriv[j] = j * x_power;
        }
        
        // Compute smoothing coefficients: c[i] = sum_j A[i][j] * (AtA^-1 * b)[j]
        smoothing_coeffs.resize(window_size);
        for (int i = 0; i < (int)window_size; ++i) {
            smoothing_coeffs[i] = 0.0f;
            for (int j = 0; j <= n; ++j) {
                float temp = 0.0f;
                for (int k = 0; k <= n; ++k) {
                    temp += inv_AtA[j][k] * b_smooth[k];
                }
                smoothing_coeffs[i] += A[i][j] * temp;
            }
        }
        
        // Compute derivative coefficients
        derivative_coeffs.resize(window_size);
        for (int i = 0; i < (int)window_size; ++i) {
            derivative_coeffs[i] = 0.0f;
            for (int j = 0; j <= n; ++j) {
                float temp = 0.0f;
                for (int k = 0; k <= n; ++k) {
                    temp += inv_AtA[j][k] * b_deriv[k];
                }
                derivative_coeffs[i] += A[i][j] * temp;
            }
        }
        
        coefficients_initialized = true;
    }
    
    void initializeCoefficients() {
        // Use computed coefficients for all window sizes
        computeSavitzkyGolayCoefficients();
        return;
        
        // Legacy pre-calculated coefficients (kept for reference but not used)
        if (window_size == 7 && polynomial_order == 2) {
            // Derivative coefficients (normalized)
            derivative_coeffs = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};
            // Smoothing coefficients
            smoothing_coeffs = {-2.0f, 3.0f, 6.0f, 7.0f, 6.0f, 3.0f, -2.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 21.0f; // Normalize
            coefficients_initialized = true;
        }
        else if (window_size == 5 && polynomial_order == 2) {
            derivative_coeffs = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
            smoothing_coeffs = {-3.0f, 12.0f, 17.0f, 12.0f, -3.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 35.0f;
            coefficients_initialized = true;
        }
        else if (window_size == 9 && polynomial_order == 2) {
            derivative_coeffs = {-4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
            smoothing_coeffs = {-21.0f, 14.0f, 39.0f, 54.0f, 59.0f, 54.0f, 39.0f, 14.0f, -21.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 231.0f;
            coefficients_initialized = true;
        }
        else if (window_size == 11 && polynomial_order == 2) {
            derivative_coeffs = {-5.0f, -4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
            smoothing_coeffs = {-36.0f, 9.0f, 44.0f, 69.0f, 84.0f, 89.0f, 84.0f, 69.0f, 44.0f, 9.0f, -36.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 429.0f;
            coefficients_initialized = true;
        }
        else if (window_size == 15 && polynomial_order == 2) {
            derivative_coeffs = {-7.0f, -6.0f, -5.0f, -4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
            smoothing_coeffs = {-78.0f, -13.0f, 42.0f, 87.0f, 122.0f, 147.0f, 162.0f, 167.0f, 162.0f, 147.0f, 122.0f, 87.0f, 42.0f, -13.0f, -78.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 1105.0f;
            coefficients_initialized = true;
        }
        else if (window_size == 21 && polynomial_order == 2) {
            derivative_coeffs = {-10.0f, -9.0f, -8.0f, -7.0f, -6.0f, -5.0f, -4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
            smoothing_coeffs = {-171.0f, -76.0f, 9.0f, 84.0f, 149.0f, 204.0f, 249.0f, 284.0f, 309.0f, 324.0f, 329.0f, 324.0f, 309.0f, 284.0f, 249.0f, 204.0f, 149.0f, 84.0f, 9.0f, -76.0f, -171.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 3059.0f;
            coefficients_initialized = true;
        }
        else if (window_size == 33 && polynomial_order == 2) {
            derivative_coeffs = {-16.0f, -15.0f, -14.0f, -13.0f, -12.0f, -11.0f, -10.0f, -9.0f, -8.0f, -7.0f, -6.0f, -5.0f, -4.0f, -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f};
            smoothing_coeffs = {-528.0f, -351.0f, -208.0f, -99.0f, -24.0f, 17.0f, 64.0f, 87.0f, 96.0f, 91.0f, 72.0f, 39.0f, -8.0f, -69.0f, -144.0f, -233.0f, -336.0f, -233.0f, -144.0f, -69.0f, -8.0f, 39.0f, 72.0f, 91.0f, 96.0f, 87.0f, 64.0f, 17.0f, -24.0f, -99.0f, -208.0f, -351.0f, -528.0f};
            for (auto& coeff : smoothing_coeffs) coeff /= 8059.0f;
            coefficients_initialized = true;
        }
    }
};

/**
 * Linear Regression Filter for trend analysis
 * Ideal for conductance filtering where trends matter more than instantaneous values
 */
class LinearRegressionFilter : public DerivativeFilter {
private:
    std::deque<float> samples;
    std::deque<float> timestamps;
    size_t max_window_size;
    float current_slope;
    float current_intercept;
    float current_timestamp;
    bool is_valid;

public:
    LinearRegressionFilter(size_t window_size = 10) 
        : max_window_size(window_size), current_slope(0.0f), current_intercept(0.0f), 
          current_timestamp(0.0f), is_valid(false) {}
    
    void addSample(float value, float timestamp = 0.0f) override {
        samples.push_back(value);
        timestamps.push_back(timestamp);
        current_timestamp = timestamp;
        
        if (samples.size() > max_window_size) {
            samples.pop_front();
            timestamps.pop_front();
        }
        
        if (samples.size() >= 3) { // Minimum for meaningful regression
            calculateRegression();
        }
    }
    
    float getDerivative(float dt) const override {
        return is_valid ? current_slope : 0.0f;
    }
    
    float getFilteredValue() const override {
        if (!is_valid || samples.empty()) {
            return samples.empty() ? 0.0f : samples.back();
        }
        // Return the predicted value at current timestamp
        return current_intercept + current_slope * current_timestamp;
    }
    
    bool isReady() const override { 
        return is_valid && samples.size() >= 3; 
    }
    
    void reset() override {
        samples.clear();
        timestamps.clear();
        current_slope = 0.0f;
        current_intercept = 0.0f;
        current_timestamp = 0.0f;
        is_valid = false;
    }
    
    const char* getTypeName() const override { return "LinearRegression"; }
    
    // Additional methods specific to linear regression
    float getSlope() const { return current_slope; }
    float getIntercept() const { return current_intercept; }
    float getRSquared() const {
        if (!is_valid || samples.size() < 3) return 0.0f;
        
        // Calculate R-squared for goodness of fit
        float sum_y = 0.0f;
        for (float sample : samples) sum_y += sample;
        float mean_y = sum_y / samples.size();
        
        float ss_tot = 0.0f, ss_res = 0.0f;
        for (size_t i = 0; i < samples.size(); ++i) {
            float y_pred = current_intercept + current_slope * timestamps[i];
            ss_tot += (samples[i] - mean_y) * (samples[i] - mean_y);
            ss_res += (samples[i] - y_pred) * (samples[i] - y_pred);
        }
        
        return ss_tot > 0.0f ? 1.0f - (ss_res / ss_tot) : 0.0f;
    }

private:
    void calculateRegression() {
        if (samples.size() < 3) {
            is_valid = false;
            return;
        }
        
        float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        size_t n = samples.size();
        
        for (size_t i = 0; i < n; ++i) {
            sum_x += timestamps[i];
            sum_y += samples[i];
            sum_xy += timestamps[i] * samples[i];
            sum_x2 += timestamps[i] * timestamps[i];
        }
        
        float denominator = n * sum_x2 - sum_x * sum_x;
        if (std::abs(denominator) > 1e-10f) {
            current_slope = (n * sum_xy - sum_x * sum_y) / denominator;
            current_intercept = (sum_y - current_slope * sum_x) / n;
            is_valid = true;
        } else {
            is_valid = false;
        }
    }
};

/**
 * Factory class for creating filter instances
 */
class FilterFactory {
public:
    enum FilterType {
        SAVITZKY_GOLAY,
        LINEAR_REGRESSION
    };
    
    // Generic factory method - caller chooses filter type
    static std::unique_ptr<DerivativeFilter> createFilter(FilterType type, size_t window_size = 7, int polynomialOrder = 2) {
        switch (type) {
            case SAVITZKY_GOLAY:
                return std::make_unique<SavitzkyGolayFilter>(window_size, polynomialOrder);
            case LINEAR_REGRESSION:
                return std::make_unique<LinearRegressionFilter>(window_size);
            default:
                return nullptr;
        }
    }
    
    // Convenience methods with specific filter types (for backward compatibility)
    static std::unique_ptr<DerivativeFilter> createSavitzkyGolayFilter(int windowSize = 7, int polynomialOrder = 2) {
        return std::make_unique<SavitzkyGolayFilter>(windowSize, polynomialOrder);
    }
    
    static std::unique_ptr<DerivativeFilter> createLinearRegressionFilter(int windowSize = 12) {
        return std::make_unique<LinearRegressionFilter>(windowSize);
    }
};

#endif // IMPROVED_FILTERING_H

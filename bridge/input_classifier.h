/**
 * Input Pattern Classifier
 *
 * Analyzes incoming mouse movement commands over a sliding window to
 * detect whether the input is pre-humanized (many small moves) or raw
 * (fewer large moves). Used to auto-select the appropriate humanization
 * mode on the KMBox.
 *
 * Classification:
 *   PRE_HUMANIZED: avg_magnitude < 8 AND move_count > 50 per second
 *   RAW:           avg_magnitude >= 8 OR move_count <= 50
 *
 * Hysteresis: 3 consecutive matching windows before switching.
 */

#ifndef INPUT_CLASSIFIER_H
#define INPUT_CLASSIFIER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    INPUT_UNKNOWN = 0,
    INPUT_PRE_HUMANIZED,
    INPUT_RAW
} input_class_t;

// Thresholds
#define IC_MAG_THRESHOLD       8    // Average |dx|+|dy| below this → pre-humanized
#define IC_COUNT_THRESHOLD     50   // Moves/sec above this → pre-humanized
#define IC_HYSTERESIS_COUNT    3    // Consecutive matching windows before switch

// Sliding window state (reset each evaluation period)
static uint32_t ic_move_count = 0;
static uint32_t ic_total_magnitude = 0;

// Classification state
static input_class_t ic_current_class = INPUT_UNKNOWN;
static input_class_t ic_pending_class = INPUT_UNKNOWN;
static uint8_t ic_hysteresis = 0;

/**
 * Record a mouse movement for classification.
 * Call this for every move command the bridge forwards to the KMBox.
 */
static inline void input_classifier_record_move(int16_t dx, int16_t dy) {
    ic_move_count++;
    // Use integer abs
    int32_t ax = dx < 0 ? -(int32_t)dx : dx;
    int32_t ay = dy < 0 ? -(int32_t)dy : dy;
    ic_total_magnitude += (uint32_t)(ax + ay);
}

/**
 * Evaluate the current window and return the classification.
 * Call this once per second (or per evaluation period).
 * Resets the window counters after evaluation.
 *
 * @return Current classification (may lag due to hysteresis)
 */
static inline input_class_t input_classifier_evaluate(void) {
    input_class_t detected;

    if (ic_move_count == 0) {
        // No data — keep current classification
        return ic_current_class;
    }

    uint32_t avg_mag = ic_total_magnitude / ic_move_count;

    if (avg_mag < IC_MAG_THRESHOLD && ic_move_count > IC_COUNT_THRESHOLD) {
        detected = INPUT_PRE_HUMANIZED;
    } else {
        detected = INPUT_RAW;
    }

    // Reset window
    ic_move_count = 0;
    ic_total_magnitude = 0;

    // Hysteresis
    if (detected == ic_pending_class) {
        ic_hysteresis++;
    } else {
        ic_pending_class = detected;
        ic_hysteresis = 1;
    }

    if (ic_hysteresis >= IC_HYSTERESIS_COUNT && ic_pending_class != ic_current_class) {
        ic_current_class = ic_pending_class;
    }

    return ic_current_class;
}

/**
 * Get current classification without evaluating.
 */
static inline input_class_t input_classifier_get(void) {
    return ic_current_class;
}

/**
 * Reset classifier state.
 */
static inline void input_classifier_reset(void) {
    ic_move_count = 0;
    ic_total_magnitude = 0;
    ic_current_class = INPUT_UNKNOWN;
    ic_pending_class = INPUT_UNKNOWN;
    ic_hysteresis = 0;
}

#endif // INPUT_CLASSIFIER_H

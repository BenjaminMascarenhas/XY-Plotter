#include "GcodeToken.h"

// Parse a G-code token into its letter and numeric value.
// For feedrate tokens, clamp the value to the allowed speed range.
void GcodeToken::Set(String token) {
    
    letter_ = token[0];                   // the command letter, such as G, X, Y, or F

    if (letter_ == 'F') {
        int f = token.substring(1).toInt(); // extract the feedrate value after the letter
        f = (f <= MIN_SPEED) ? MIN_SPEED : f; // prevent speeds below the allowed minimum
        value_ = (f >= MAX_SPEED) ? MAX_SPEED : f; // prevent speeds above the allowed maximum
    } else {
        value_ = token.substring(1).toInt();  // convert the numeric part after the letter to an int
    }

}
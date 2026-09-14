#ifndef GCODETOKEN_H
#define GCODETOKEN_H

#include <Arduino.h>

#define MAX_TOKENS 4
#define MAX_SPEED 1000    // max speed
#define MIN_SPEED 300

class GcodeToken {
  private:
    char letter_;
    int value_;

  public:
    GcodeToken() : letter_(' '), value_(0) {};  // Constructor
    void Set(String token);                     // Set the token
    char GetLetter() { return letter_; }        // Getter for letter
    int GetValue() { return value_; }           // Getter for value
    //bool CheckIfValid();                        // check token is valid i.e doesnt go out of bounds

};
#endif